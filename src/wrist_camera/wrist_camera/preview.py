"""腕部相机纯 Python 按需拉流 MJPEG 网页预览服务（不依赖 ROS；没人打开网页时不拉流）。"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from wrist_camera.rtsp import (
    RtspReader,
    default_cameras_config_path,
    ffmpeg_command,
    probe_size,
    resolve_camera_selection,
    resolve_stream_url,
)

_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PiPER-X 腕部相机预览</title>
<style>
  body { margin: 0; padding: 18px; background: #101216; color: #e5e7eb; font-family: system-ui, sans-serif; }
  header { display: flex; align-items: baseline; gap: 14px; margin-bottom: 16px; }
  h1 { margin: 0; font-size: 18px; }
  .hint { color: #9ca3af; font-size: 13px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(460px, 1fr)); gap: 16px; }
  .card { background: #181b22; border: 1px solid #282d37; border-radius: 10px; overflow: hidden; }
  .head { display: flex; align-items: center; gap: 10px; padding: 10px 14px; border-bottom: 1px solid #242933; }
  .title { font-weight: 600; font-size: 14px; }
  .grow { flex: 1; }
  select { background: #101216; color: #e5e7eb; border: 1px solid #374151; border-radius: 6px; padding: 4px 8px; }
  .status { font-size: 12px; color: #9ca3af; padding: 8px 14px; }
  img { display: block; width: 100%; background: #090a0d; min-height: 240px; object-fit: contain; }
</style>
</head>
<body>
<header>
  <h1>PiPER-X 腕部相机实时预览</h1>
  <span class="hint">按需拉流：关闭页面后自动断开 RTSP，平时零带宽 / 零解码开销</span>
</header>
<div id="grid" class="grid"></div>
<script>
const DEFAULT_STREAM = __DEFAULT_STREAM__;
const CAMERAS = __CAMERAS__;
const grid = document.getElementById('grid');

CAMERAS.forEach((cam) => {
  const card = document.createElement('div');
  card.className = 'card';
  card.innerHTML = `
    <div class="head">
      <span class="title">${cam.label} (${cam.name} · ${cam.ip})</span>
      <span class="grow"></span>
      <select data-cam="${cam.name}">
        <option value="stream1">720×480 (stream1)</option>
        <option value="stream0">1920×1080 (stream0)</option>
      </select>
    </div>
    <img id="img-${cam.name}" alt="${cam.label}">
    <div class="status" id="st-${cam.name}">连接中…</div>`;
  grid.appendChild(card);
  const sel = card.querySelector('select');
  sel.value = DEFAULT_STREAM;
  const img = card.querySelector('img');
  const bind = () => {
    img.src = `/video_feed?camera=${encodeURIComponent(cam.name)}&stream=${encodeURIComponent(sel.value)}&t=${Date.now()}`;
  };
  sel.onchange = bind;
  bind();
});

async function poll() {
  try {
    const res = await fetch('/status');
    const data = await res.json();
    (data.cameras || []).forEach((item) => {
      const el = document.getElementById('st-' + item.name);
      if (el) el.textContent = item.message;
    });
  } catch (_) {}
}
setInterval(poll, 2000);
poll();
</script>
</body>
</html>
"""


def connected(sock: socket.socket) -> bool:
    """检测浏览器对端连接是否仍在（无新帧期间靠它及时感知页面关闭）。"""
    try:
        return sock.recv(1, socket.MSG_DONTWAIT | socket.MSG_PEEK) != b''
    except BlockingIOError:
        return True
    except OSError:
        return False


class _PreviewSlot:
    """单台腕部相机的按需 RTSP 拉流槽：有浏览器连接时才拉流，连接归零自动停流。"""

    def __init__(
        self,
        entry: dict[str, Any],
        *,
        default_stream: str = 'stream1',
        jpeg_quality: int = 75,
        poll_period_s: float = 1.0,
        stale_timeout_s: float = 5.0,
        config_file: str | Path | None = None,
        reader_cls=RtspReader,
        probe_fn=probe_size,
    ) -> None:
        self.entry = entry
        self.name = str(entry['name'])
        self.label = str(entry.get('label', self.name))
        self.ip = str(entry.get('ip', ''))
        self.jpeg_quality = int(jpeg_quality)
        self._config_file = config_file
        self._poll_period = poll_period_s
        self._stale_timeout = stale_timeout_s
        self._reader_cls = reader_cls
        self._probe_fn = probe_fn

        self._cond = threading.Condition()
        self._wake = threading.Event()
        self._stopping = threading.Event()
        self._viewers = 0
        self._stream = default_stream
        self._applied_stream: str | None = None
        self._reader: RtspReader | None = None
        self._frame: tuple[int, bytes, int, int] | None = None
        self._seq = 0
        self._error: str | None = None
        self._supervisor = threading.Thread(
            target=self._supervise, daemon=True, name=f'preview-{self.name}')
        self._supervisor.start()

    @property
    def stopping(self) -> bool:
        return self._stopping.is_set()

    def add_viewer(self, stream: str = '') -> None:
        with self._cond:
            self._viewers += 1
            if stream in ('stream0', 'stream1'):
                self._stream = stream
        self._wake.set()

    def remove_viewer(self) -> None:
        with self._cond:
            self._viewers = max(0, self._viewers - 1)
        self._wake.set()

    def wait_frame(self, after_seq: int, timeout: float = 1.0) -> tuple[int, bytes, int, int] | None:
        with self._cond:
            if self._frame is None or self._frame[0] <= after_seq:
                self._cond.wait(timeout)
            frame = self._frame
        if frame is None or frame[0] <= after_seq:
            return None
        return frame

    def status(self) -> dict[str, Any]:
        with self._cond:
            viewers = self._viewers
            stream = self._stream
        reader = self._reader
        if self._error is not None:
            return {'name': self.name, 'ip': self.ip, 'state': 'error',
                    'viewers': viewers, 'message': f'异常：{self._error}'}
        if reader is None:
            return {'name': self.name, 'ip': self.ip, 'state': 'idle',
                    'viewers': viewers, 'message': '空闲：无人观看，未拉流'}
        return {
            'name': self.name, 'ip': self.ip, 'state': 'streaming',
            'stream': stream, 'viewers': viewers,
            'width': reader.width, 'height': reader.height,
            'message': f'{stream} · {reader.width}×{reader.height} · 最近一帧 {reader.frame_age():.1f}s 前（观众 {viewers}）',
        }

    def _on_frame(self, raw: bytes, width: int, height: int) -> None:
        self._error = None
        with self._cond:
            self._seq += 1
            self._frame = (self._seq, raw, width, height)
            self._cond.notify_all()

    def _supervise(self) -> None:
        while not self._stopping.is_set():
            self._wake.clear()
            try:
                self._step()
            except Exception as exc:
                self._error = str(exc)
            if self._stopping.is_set():
                break
            self._wake.wait(self._poll_period)

    def _step(self) -> None:
        with self._cond:
            viewers = self._viewers
            stream = self._stream
        reader = self._reader
        if viewers <= 0:
            if reader is not None:
                self._close_reader()
            return
        if (reader is None or not reader.alive()
                or reader.frame_age() > self._stale_timeout
                or self._applied_stream != stream):
            self._close_reader()
            self._open_reader(stream)

    def _open_reader(self, stream: str) -> None:
        if self._stopping.is_set():
            return
        url, native = resolve_stream_url(self.name, stream=stream, config_file=self._config_file)
        if native is None:
            native = self._probe_fn(url)
        if native is None:
            self._error = f'无法探测分辨率：{url}'
            return
        reader = self._reader_cls(ffmpeg_command(url), native[0], native[1], self._on_frame)
        if self._stopping.is_set():
            reader.stop()
            return
        self._reader = reader
        self._applied_stream = stream
        self._error = None

    def _close_reader(self) -> None:
        reader = self._reader
        self._reader = None
        self._applied_stream = None
        if reader is not None:
            reader.stop()

    def close(self) -> None:
        self._stopping.set()
        self._wake.set()
        self._supervisor.join(timeout=3.0)
        self._close_reader()
        with self._cond:
            self._cond.notify_all()


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'

    def log_message(self, *_args) -> None:
        pass

    def do_GET(self) -> None:  # noqa: N802
        srv: PreviewServer = self.server  # type: ignore[assignment]
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if parsed.path == '/video_feed':
            cam = (query.get('camera') or [''])[0]
            stream = (query.get('stream') or [''])[0]
            slot = srv.resolve_slot(cam)
            if slot is None:
                self.send_error(404)
                return
            self._stream(slot, stream)
        elif parsed.path == '/status':
            payload = {
                'cameras': [s.status() for s in srv.slots.values()],
                'message': ' | '.join(f"{s.name}: {s.status()['message']}" for s in srv.slots.values()),
            }
            self._send('application/json; charset=utf-8',
                       json.dumps(payload, ensure_ascii=False).encode('utf-8'))
        elif parsed.path in ('/', '/index.html'):
            self._send('text/html; charset=utf-8', srv.page_html)
        else:
            self.send_error(404)

    def _send(self, content_type: str, body: bytes) -> None:
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _stream(self, slot: _PreviewSlot, stream: str) -> None:
        self.send_response(200)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        slot.add_viewer(stream)
        seq = 0
        try:
            while not slot.stopping:
                frame = slot.wait_frame(seq, 1.0)
                if frame is None:
                    if not connected(self.connection):
                        break
                    continue
                seq, raw, width, height = frame
                image = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
                ok, jpeg = cv2.imencode(
                    '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, slot.jpeg_quality])
                if ok:
                    data = jpeg.tobytes()
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(data)}\r\n\r\n'.encode('ascii')
                        + data + b'\r\n'
                    )
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            slot.remove_viewer()


class PreviewServer(ThreadingHTTPServer):
    """按需拉流的双臂/单臂腕部相机 MJPEG 预览服务器。"""

    daemon_threads = True

    def __init__(
        self,
        port: int = 8010,
        *,
        host: str = '0.0.0.0',
        arm: str = '',
        camera: str = '',
        stream: str = 'stream1',
        jpeg_quality: int = 75,
        config_file: str | Path | None = None,
        reader_cls=RtspReader,
        probe_fn=probe_size,
    ) -> None:
        selected = resolve_camera_selection(arm=arm, camera=camera, config_file=config_file)
        self.slots: dict[str, _PreviewSlot] = {
            entry['name']: _PreviewSlot(
                entry,
                default_stream=stream,
                jpeg_quality=jpeg_quality,
                config_file=config_file,
                reader_cls=reader_cls,
                probe_fn=probe_fn,
            )
            for entry in selected
        }
        cam_meta = [
            {'name': e['name'], 'label': e.get('label', e['name']), 'ip': e.get('ip', '')}
            for e in selected
        ]
        html = (
            _PAGE.replace('__DEFAULT_STREAM__', json.dumps(stream))
            .replace('__CAMERAS__', json.dumps(cam_meta, ensure_ascii=False))
        )
        self.page_html = html.encode('utf-8')
        super().__init__((host, port), _Handler)
        self._thread = threading.Thread(
            target=self.serve_forever, daemon=True, name='wrist-camera-preview')
        self._thread.start()

    def resolve_slot(self, selector: str) -> _PreviewSlot | None:
        if not selector:
            return next(iter(self.slots.values()), None)
        if selector in self.slots:
            return self.slots[selector]
        low = selector.strip().lower().strip('/')
        for slot in self.slots.values():
            aliases = {a.lower().strip('/') for a in slot.entry.get('aliases', ())}
            if low in aliases or low in {slot.name.lower(), slot.ip.lower(), str(slot.entry.get('arm', '')).lower()}:
                return slot
        return None

    def stop(self) -> None:
        for slot in self.slots.values():
            slot.close()
        self.shutdown()
        self.server_close()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description='PiPER-X 腕部相机按需拉流网页预览（不传 --arm/--camera 时默认同时挂载双臂）')
    parser.add_argument('--arm', default='', help='可选：arm_3 / arm_4 / all（默认同时挂载双臂）')
    parser.add_argument('--camera', default='', help='可选：camera_arm_3 / cam4 / IP 等别名')
    parser.add_argument('--stream', default='stream1', choices=['stream1', 'stream0'],
                        help='默认码流档位：stream1 (720x480) 或 stream0 (1920x1080)')
    parser.add_argument('--port', type=int, default=8010)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--jpeg-quality', type=int, default=75)
    parser.add_argument('--config-file', default=str(default_cameras_config_path()))
    args = parser.parse_args(argv)

    server = PreviewServer(
        args.port,
        host=args.host,
        arm=args.arm,
        camera=args.camera,
        stream=args.stream,
        jpeg_quality=args.jpeg_quality,
        config_file=args.config_file or None,
    )
    names = ', '.join(f'{s.name} ({s.ip})' for s in server.slots.values())
    print(f'腕部相机预览已就绪: http://127.0.0.1:{server.server_port}  [{names}]（打开网页才拉流）', flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
