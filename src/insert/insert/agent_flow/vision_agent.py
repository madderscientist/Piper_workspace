"""Single-call dual-image numeric perception via local copilot-api (gpt-6.1-sol)."""

from __future__ import annotations

import base64
import json
import threading
import time
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np

DEFAULT_API_BASE = 'http://127.0.0.1:4141'
DEFAULT_MODEL = 'gpt-6.1-sol'
DEFAULT_REASONING_EFFORT = 'max'

DEFAULT_OBJECT_PROMPT = '把黑色夹爪捏着的插头的末端，和对应插孔标记出来。'

DEFAULT_OUTPUT_PROMPT = (
    '输入为两张 1920×1080 外部相机原图：第 1 张为 cam1 (head_1)，第 2 张为 cam2 (head_2)，'
    '左上角为 [0, 0]，u 向右 (0~1919)，v 向下 (0~1079)。\n'
    '只输出纯数值 JSON，禁止输出任何文字描述：\n'
    '- symmetry_deg：截面旋转对称周期角度（圆形填 0，正六边形填 60，正方形填 90，长方形填 180，非对称形状填 360）；\n'
    '- cam1 与 cam2 各自输出同一组物理目标的 6 个像素坐标 [u, v]：\n'
    '  1. plug_tip：插头最前端插合端面正中心 [u, v]；\n'
    '  2. plug_axis_point：从 plug_tip 沿插头杆身轴线向夹爪方向回退约 80 像素的点 [u, v]；\n'
    '  3. plug_orient_point：从 plug_tip 沿插头端面主特征方向（如长边方向）延伸约 60 像素的点 [u, v]；\n'
    '  4. hole_center：与该插头截面匹配的对应插孔入口正中心 [u, v]；\n'
    '  5. hole_insert_point：从 hole_center 沿插头插入孔内的三维孔轴正方向延伸约 80 像素的点 [u, v]；\n'
    '  6. hole_orient_point：从 hole_center 沿插孔端面主特征方向（与 plug_orient_point 对应的长边/特征方向）延伸约 60 像素的点 [u, v]。'
)

_COORD_PAIR_SCHEMA: dict[str, Any] = {
    'type': 'array',
    'items': {'type': 'number'},
    'minItems': 2,
    'maxItems': 2,
}

_VIEW_SCHEMA: dict[str, Any] = {
    'type': 'object',
    'properties': {
        'plug_tip': _COORD_PAIR_SCHEMA,
        'plug_axis_point': _COORD_PAIR_SCHEMA,
        'plug_orient_point': _COORD_PAIR_SCHEMA,
        'hole_center': _COORD_PAIR_SCHEMA,
        'hole_insert_point': _COORD_PAIR_SCHEMA,
        'hole_orient_point': _COORD_PAIR_SCHEMA,
    },
    'required': [
        'plug_tip',
        'plug_axis_point',
        'plug_orient_point',
        'hole_center',
        'hole_insert_point',
        'hole_orient_point',
    ],
    'additionalProperties': False,
}

PERCEPTION_JSON_SCHEMA: dict[str, Any] = {
    'type': 'object',
    'properties': {
        'symmetry_deg': {'type': 'number'},
        'cam1': _VIEW_SCHEMA,
        'cam2': _VIEW_SCHEMA,
    },
    'required': ['symmetry_deg', 'cam1', 'cam2'],
    'additionalProperties': False,
}


class PerceptionCancelledError(RuntimeError):
    pass


def _encode_image_data_url(image_bgr: np.ndarray, max_bytes: int = 2_800_000) -> str:
    for quality in (90, 82, 72, 62):
        ok, buf = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError('编码相机图像为 JPEG 失败')
        raw = buf.tobytes()
        if len(raw) <= max_bytes or quality == 62:
            return 'data:image/jpeg;base64,' + base64.b64encode(raw).decode('ascii')
    raise RuntimeError('图像大小超出限制')


def _extract_response_text(payload: dict[str, Any]) -> str:
    direct = payload.get('output_text')
    if isinstance(direct, str) and direct.strip():
        return direct.strip()
    for item in payload.get('output') or []:
        if not isinstance(item, dict) or item.get('type') != 'message':
            continue
        for block in item.get('content') or []:
            if isinstance(block, dict) and block.get('type') in ('output_text', 'text'):
                txt = block.get('text')
                if isinstance(txt, str) and txt.strip():
                    return txt.strip()
    raise RuntimeError(f'模型未返回有效 JSON 文本: {json.dumps(payload, ensure_ascii=False)[:400]}')


def _parse_xy(pt: Any, width: int, height: int) -> list[float]:
    if isinstance(pt, dict):
        u_raw, v_raw = float(pt['u']), float(pt['v'])
    else:
        u_raw, v_raw = float(pt[0]), float(pt[1])
    u = round(max(0.0, min(float(width - 1), u_raw)), 2)
    v = round(max(0.0, min(float(height - 1), v_raw)), 2)
    return [u, v]


def _clamp_view_points(view: dict[str, Any], width: int, height: int) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for key in (
        'plug_tip',
        'plug_axis_point',
        'plug_orient_point',
        'hole_center',
        'hole_insert_point',
        'hole_orient_point',
    ):
        if key in view and view[key] is not None:
            out[key] = _parse_xy(view[key], width, height)
    if 'plug_orient_point' not in out and 'plug_tip' in out:
        out['plug_orient_point'] = [min(float(width - 1), out['plug_tip'][0] + 40.0), out['plug_tip'][1]]
    if 'hole_orient_point' not in out and 'hole_center' in out:
        out['hole_orient_point'] = [min(float(width - 1), out['hole_center'][0] + 40.0), out['hole_center'][1]]
    return out


def perceive_dual_views(
    img1_bgr: np.ndarray,
    img2_bgr: np.ndarray,
    *,
    object_prompt: str = DEFAULT_OBJECT_PROMPT,
    output_prompt: str = DEFAULT_OUTPUT_PROMPT,
    cam1_label: str = 'head_1',
    cam2_label: str = 'head_2',
    api_base: str = DEFAULT_API_BASE,
    model: str = DEFAULT_MODEL,
    reasoning_effort: str = DEFAULT_REASONING_EFFORT,
    image_detail: str = 'high',
    timeout_s: float = 360.0,
    on_progress: Callable[[dict[str, Any]], None] | None = None,
    cancel_event: threading.Event | None = None,
) -> dict[str, Any]:
    """Run a single SSE-streamed multimodal Responses API call returning pure numeric coordinates."""
    h1, w1 = img1_bgr.shape[:2]
    h2, w2 = img2_bgr.shape[:2]
    seg1 = (object_prompt or DEFAULT_OBJECT_PROMPT).strip()
    seg2 = (output_prompt or DEFAULT_OUTPUT_PROMPT).strip()
    full_prompt = (
        f'{seg1}\n\n'
        f'[cam1={cam1_label}: {w1}x{h1}, cam2={cam2_label}: {w2}x{h2}]\n'
        f'{seg2}'
    )

    body: dict[str, Any] = {
        'model': model,
        'reasoning': {'effort': reasoning_effort},
        'stream': True,
        'input': [
            {
                'role': 'user',
                'content': [
                    {'type': 'input_text', 'text': full_prompt},
                    {'type': 'input_image', 'image_url': _encode_image_data_url(img1_bgr), 'detail': image_detail},
                    {'type': 'input_image', 'image_url': _encode_image_data_url(img2_bgr), 'detail': image_detail},
                ],
            }
        ],
        'text': {
            'format': {
                'type': 'json_schema',
                'name': 'dual_view_numeric_perception',
                'strict': True,
                'schema': PERCEPTION_JSON_SCHEMA,
            }
        },
    }

    url = api_base.rstrip('/') + '/responses'
    t0 = time.perf_counter()
    req = Request(
        url,
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json', 'Accept': 'text/event-stream'},
        method='POST',
    )

    reasoning_steps = 0
    output_chunks: list[str] = []
    completed_payload: dict[str, Any] | None = None
    actual_model = model

    try:
        with urlopen(req, timeout=timeout_s) as resp:
            ctype = (resp.headers.get('Content-Type') or '').lower()
            if 'event-stream' not in ctype:
                resp_json = json.loads(resp.read().decode('utf-8'))
                raw_text = _extract_response_text(resp_json)
                actual_model = str(resp_json.get('model') or model)
            else:
                cur_event = ''
                while True:
                    if cancel_event is not None and cancel_event.is_set():
                        raise PerceptionCancelledError('已取消当前 Agent 识别请求')
                    raw_line = resp.readline()
                    if not raw_line:
                        break
                    line = raw_line.decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    if line.startswith('event:'):
                        cur_event = line.split(':', 1)[1].strip()
                        continue
                    if not line.startswith('data:'):
                        continue
                    data_str = line[5:].strip()
                    if not data_str or data_str == '[DONE]':
                        continue
                    try:
                        ev_data = json.loads(data_str)
                    except Exception:
                        continue

                    elapsed = round(time.perf_counter() - t0, 1)
                    if cur_event == 'response.output_item.done':
                        reasoning_steps += 1
                        if on_progress is not None:
                            on_progress({
                                'stage': 'reasoning',
                                'reasoning_steps': reasoning_steps,
                                'elapsed_s': elapsed,
                                'partial_output': ''.join(output_chunks),
                            })
                    elif cur_event == 'response.output_text.delta':
                        delta = str(ev_data.get('delta', ''))
                        if delta:
                            output_chunks.append(delta)
                            if on_progress is not None:
                                on_progress({
                                    'stage': 'generating',
                                    'reasoning_steps': reasoning_steps,
                                    'elapsed_s': elapsed,
                                    'partial_output': ''.join(output_chunks),
                                })
                    elif cur_event == 'response.failed':
                        err_obj = (ev_data.get('response') or {}).get('error') or ev_data
                        raise RuntimeError(f'模型推理失败: {json.dumps(err_obj, ensure_ascii=False)[:400]}')
                    elif cur_event == 'error':
                        raise RuntimeError(f'流式请求异常: {json.dumps(ev_data, ensure_ascii=False)[:400]}')
                    elif cur_event == 'response.completed':
                        completed_payload = ev_data.get('response') if isinstance(ev_data.get('response'), dict) else ev_data
                        if isinstance(completed_payload, dict) and completed_payload.get('model'):
                            actual_model = str(completed_payload['model'])
                        break

                raw_text = ''.join(output_chunks).strip()
                if not raw_text and completed_payload is not None:
                    raw_text = _extract_response_text(completed_payload)
    except HTTPError as exc:
        detail = exc.read().decode('utf-8', errors='replace')[:500]
        raise RuntimeError(f'调用 LLM 接口失败 ({exc.code}): {detail}') from exc
    except URLError as exc:
        raise RuntimeError(f'无法连接 LLM 接口 {url}: {exc.reason}') from exc

    latency_s = time.perf_counter() - t0
    parsed = json.loads(raw_text)
    symmetry_deg = float(parsed.get('symmetry_deg', 0.0))
    cam1_pts = _clamp_view_points(parsed['cam1'], w1, h1)
    cam2_pts = _clamp_view_points(parsed['cam2'], w2, h2)
    raw_numeric = {
        'symmetry_deg': symmetry_deg,
        'cam1': cam1_pts,
        'cam2': cam2_pts,
    }
    return {
        'raw_numeric': raw_numeric,
        'symmetry_deg': symmetry_deg,
        'cam1': cam1_pts,
        'cam2': cam2_pts,
        'model': actual_model,
        'reasoning_effort': reasoning_effort,
        'reasoning_steps': reasoning_steps,
        'latency_s': round(latency_s, 2),
        'object_prompt': seg1,
        'output_prompt': seg2,
    }
