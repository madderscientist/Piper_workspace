"""传感器配置解析与标定参数原子持久化工具（独立于 ROS 节点）。

- 包内 config/px6d.yaml：协议默认值与公共校准姿态（进 git）。
- $PIPER_CONFIG_DIR/paxini_driver/<传感器名>.yaml：本机传感器（arm、serial_port、expected_version、calibration 等）。
- 传感器文件中 calibration 指向的标定结果文件；缺失或无效视为未标定。
"""

from __future__ import annotations

import fcntl
import os
import tempfile
from pathlib import Path
from typing import Any, Callable, List, Optional, Sequence

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_path


def source_config_path(fallback: Optional[Path] = None) -> Path:
    """优先使用显式指定路径；默认在 symlink-install 下定位源码 config/px6d.yaml，否则回退到 share 目录。"""
    if fallback is not None:
        return Path(fallback)
    source = Path(__file__).resolve().parents[1] / "config" / "px6d.yaml"
    if source.is_file():
        return source
    try:
        return get_package_share_path("paxini_driver") / "config" / "px6d.yaml"
    except PackageNotFoundError:
        return source


def default_sensor_dir() -> Path:
    root = os.environ.get("PIPER_CONFIG_DIR")
    if not root:
        raise RuntimeError("PIPER_CONFIG_DIR 未设置，请先 source setup_env.bash")
    return Path(root) / "paxini_driver"


def _load_mapping(path: Path) -> dict[str, Any]:
    """缺失或无法解析的文件视为空。"""
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def resolve_sensor_configs(
    selector: str = "",
    config_file: Optional[str] = None,
    sensor_dir: Optional[str] = None,
) -> List[dict[str, Any]]:
    """按传感器名或机械臂 id 选择本机传感器；空或 all 表示全部。"""
    raw = yaml.safe_load(source_config_path(config_file).read_text()) or {}
    directory = Path(sensor_dir) if sensor_dir else default_sensor_dir()
    selector = selector.strip()
    entries: List[dict[str, Any]] = []
    for file in sorted(directory.glob("*.yaml")):
        cfg = yaml.safe_load(file.read_text()) or {}
        arm = str(cfg["arm"])
        if selector not in ("", "all", file.stem, arm):
            continue
        calibration = (file.parent / cfg["calibration"]).resolve() if cfg.get("calibration") else None
        entries.append(
            {
                "namespace": f"/{arm}",
                **(raw.get("defaults") or {}),
                **cfg,
                **(_load_mapping(calibration) if calibration else {}),
                "sensor": file.stem,
                "arm_id": arm,
                "calibration_file": str(calibration or ""),
                "calibration_poses": raw.get("calibration_poses") or [],
            }
        )
    if not entries:
        raise ValueError(f"No PX6D sensor matches '{selector}' in {directory}")
    return entries


def _update_yaml(target: Path, update: Callable[[dict[str, Any]], None]) -> Path:
    """文件排他锁串行化并发写入，同目录临时文件 + os.replace 保证驱动 reload 时读到完整 YAML。"""
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.with_suffix(target.suffix + ".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        raw = _load_mapping(target)
        update(raw)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=target.parent,
                prefix=f".{target.name}.",
                delete=False,
            ) as stream:
                temporary = Path(stream.name)
                yaml.safe_dump(raw, stream, allow_unicode=True, sort_keys=False)
            if target.exists():
                temporary.chmod(target.stat().st_mode & 0o777)
            os.replace(temporary, target)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return target


def save_calibration_poses(
    poses: Sequence[Sequence[float]], config_path: Optional[Path] = None
) -> Path:
    """公共校准姿态写入包内 px6d.yaml。"""
    rounded = [[round(float(v), 6) for v in pose[:6]] for pose in poses]
    return _update_yaml(
        source_config_path(config_path),
        lambda raw: raw.__setitem__("calibration_poses", rounded),
    )


def save_sensor_calibration(
    calibration_file: str,
    *,
    tare_offset: Sequence[float],
    tool_mass_kg: float,
    tool_com_m: Sequence[float],
    calibration_samples: Sequence[dict[str, Any]],
    calibration_quality: dict[str, Any],
) -> Path:
    """单个传感器的标定结果写入其 calibration 文件。"""
    if not calibration_file:
        raise ValueError("传感器配置缺少 calibration 字段，无法保存标定结果")
    values = {
        "tare_offset": [round(float(v), 6) for v in tare_offset[:6]],
        "tool_mass_kg": round(float(tool_mass_kg), 6),
        "tool_com_m": [round(float(v), 6) for v in tool_com_m[:3]],
        "calibration_samples": list(calibration_samples),
        "calibration_quality": dict(calibration_quality),
    }
    return _update_yaml(Path(calibration_file), lambda raw: raw.update(values))
