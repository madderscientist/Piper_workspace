"""Shared sensor selection and calibration persistence; independent of ROS nodes."""

from __future__ import annotations

import fcntl
import os
import tempfile
from pathlib import Path
from typing import Any, List, Optional, Sequence

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_path


def source_config_path(fallback: Optional[Path] = None) -> Path:
    """Use an explicit path, source config for symlink installs, or installed share."""
    if fallback is not None:
        return Path(fallback)
    source = Path(__file__).resolve().parents[1] / "config" / "px6d.yaml"
    if source.is_file():
        return source
    try:
        return get_package_share_path("paxini_driver") / "config" / "px6d.yaml"
    except PackageNotFoundError:
        return source


def default_config_path() -> Path:
    return source_config_path()


def resolve_sensor_configs(
    arm: str = "",
    config_file: Optional[str] = None,
) -> List[dict[str, Any]]:
    path = Path(config_file) if config_file else default_config_path()
    raw = yaml.safe_load(path.read_text()) or {}
    defaults = dict(raw.get("defaults") or {})
    sensors = raw.get("sensors") or {}
    selector = arm.strip().lower()
    entries: List[dict[str, Any]] = []
    for key, cfg in sensors.items():
        merged = {
            **defaults,
            **cfg,
            "arm_id": key,
            "calibration_poses": raw.get("calibration_poses") or [],
        }
        aliases = {str(a).strip().lower() for a in merged.get("aliases", [])} | {
            key.lower()
        }
        if not selector or selector == "all" or selector in aliases:
            entries.append(merged)
    if not entries:
        raise ValueError(
            f"Unknown sensor arm selector '{arm}'; available: {list(sensors)}"
        )
    return entries


def save_sensor_calibration_to_yaml(
    arm_id: str,
    *,
    tare_offset: Optional[Sequence[float]] = None,
    tool_mass_kg: Optional[float] = None,
    tool_com_m: Optional[Sequence[float]] = None,
    calibration_poses: Optional[Sequence[Sequence[float]]] = None,
    calibration_samples: Optional[Sequence[dict[str, Any]]] = None,
    calibration_quality: Optional[dict[str, Any]] = None,
    config_path: Optional[Path] = None,
) -> Path:
    """Persist shared poses and per-sensor calibration parameters in px6d.yaml."""
    target = source_config_path(config_path).resolve()
    # Serialize independent arm updates and let reload readers see complete YAML.
    with target.with_suffix(target.suffix + ".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        raw = yaml.safe_load(target.read_text(encoding="utf-8")) or {}
        sensors = raw.setdefault("sensors", {})
        entry = sensors.setdefault(arm_id, {})
        if tare_offset is not None:
            entry["tare_offset"] = [round(float(v), 6) for v in tare_offset[:6]]
        if tool_mass_kg is not None:
            entry["tool_mass_kg"] = round(float(tool_mass_kg), 6)
        if tool_com_m is not None:
            entry["tool_com_m"] = [round(float(v), 6) for v in tool_com_m[:3]]
        if calibration_samples is not None:
            entry["calibration_samples"] = list(calibration_samples)
        if calibration_quality is not None:
            entry["calibration_quality"] = dict(calibration_quality)
        if calibration_poses is not None:
            raw["calibration_poses"] = [
                [round(float(v), 6) for v in pose[:6]] for pose in calibration_poses
            ]
            for sensor in sensors.values():
                sensor.pop("calibration_poses", None)
        content = yaml.safe_dump(raw, allow_unicode=True, sort_keys=False)
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
                stream.write(content)
            temporary.chmod(target.stat().st_mode & 0o777)
            os.replace(temporary, target)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return target
