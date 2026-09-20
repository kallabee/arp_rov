from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml


class SensorMode:
    """Tri-state sensor intent under ``sensors.<id>.mode``."""

    DISABLED = "disabled"
    ENABLED = "enabled"
    ENABLED_IF_CAN = "enabled_if_can"
    _ALL = (DISABLED, ENABLED, ENABLED_IF_CAN)


def resolve_sensor_mode(s: Mapping[str, Any]) -> str:
    """Require explicit ``mode`` for each sensor block (``disabled`` | ``enabled`` | ``enabled_if_can``)."""
    if not isinstance(s, Mapping):
        return SensorMode.DISABLED
    if "mode" not in s:
        raise ValueError(
            "Each entry under ``sensors`` must set ``mode`` to one of: "
            f"{list(SensorMode._ALL)} (got keys: {list(s.keys())!r})."
        )
    m = str(s["mode"]).lower().strip()
    if m not in SensorMode._ALL:
        raise ValueError(f"Invalid sensors.mode {m!r}; expected one of {list(SensorMode._ALL)}.")
    return m


def normalize_sensor_entry(d: Any) -> Dict[str, Any]:
    """Return a copy of a sensor mapping with validated ``mode``."""
    if not isinstance(d, dict):
        return {"mode": SensorMode.DISABLED}
    out = dict(d)
    out["mode"] = resolve_sensor_mode(out)
    return out


@dataclass(frozen=True)
class SensorFlags:
    """Resolved tri-state for one logical sensor."""

    mode: str

    def wants_init(self) -> bool:
        return self.mode != SensorMode.DISABLED

    def init_raises_on_failure(self) -> bool:
        """If opening the device fails, startup should abort (strict)."""
        return self.mode == SensorMode.ENABLED

    def wants_samples(self) -> bool:
        """Whether we intend to read this sensor when the driver instance exists."""
        return self.mode in (SensorMode.ENABLED, SensorMode.ENABLED_IF_CAN)


@dataclass(frozen=True)
class MonitorValueConfig:
    log_dir: Path
    log_rotate_seconds: float
    sample_interval_seconds: float
    imu_frame_id: str
    sensors: Dict[str, Dict[str, Any]]
    imu_nav: Dict[str, Any] = field(default_factory=dict)
    # TensorBoard event files (view with: tensorboard --logdir <tensorboard_log_dir>)
    tensorboard_enabled: bool = True
    tensorboard_log_dir: Optional[Path] = None

    def sensor_flags(self, sensor_id: str) -> SensorFlags:
        s = self.sensors.get(sensor_id, {}) if isinstance(self.sensors, dict) else {}
        mode = str(s.get("mode", SensorMode.DISABLED))
        return SensorFlags(mode=mode)

    def sensor_init_raises(self, sensor_id: str) -> bool:
        """Whether a failed sensor *initialization* should abort startup (``mode: enabled``)."""
        return self.sensor_flags(sensor_id).init_raises_on_failure()

    def sensor_cfg(self, sensor_id: str) -> Dict[str, Any]:
        s = self.sensors.get(sensor_id, {}) if isinstance(self.sensors, dict) else {}
        return dict(s) if isinstance(s, dict) else {}


def _normalize_sensors_map(sensors: Any) -> Dict[str, Dict[str, Any]]:
    if not isinstance(sensors, dict):
        raise ValueError("sensors must be a mapping")
    return {str(k): normalize_sensor_entry(v or {}) for k, v in sensors.items()}


def load_config(path: str | Path) -> MonitorValueConfig:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config must be a mapping: {p}")

    log_dir = Path(raw.get("log_dir", "./log/monitor_value"))
    log_rotate_seconds = float(raw.get("log_rotate_seconds", 1800))
    sample_interval_seconds = float(raw.get("sample_interval_seconds", 1.0))
    imu_frame_id = str(raw.get("imu_frame_id", "imu_link"))
    sensors = _normalize_sensors_map(raw.get("sensors", {}) or {})

    imu_nav = raw.get("imu_nav") or {}
    if not isinstance(imu_nav, dict):
        raise ValueError("imu_nav must be a mapping when present")

    # Default: TensorBoard on. Explicit false / 0 / "off" disables.
    tb_raw = raw.get("tensorboard_enabled", True)
    if isinstance(tb_raw, str):
        tensorboard_enabled = tb_raw.strip().lower() not in ("0", "false", "no", "off", "")
    else:
        tensorboard_enabled = bool(tb_raw)

    tb_dir_raw = raw.get("tensorboard_log_dir", None)
    if tb_dir_raw is None or str(tb_dir_raw).strip() == "":
        tensorboard_log_dir: Optional[Path] = log_dir / "tb"
    else:
        tensorboard_log_dir = Path(str(tb_dir_raw))

    return MonitorValueConfig(
        log_dir=log_dir,
        log_rotate_seconds=log_rotate_seconds,
        sample_interval_seconds=sample_interval_seconds,
        imu_frame_id=imu_frame_id,
        sensors=sensors,
        imu_nav=copy.deepcopy(imu_nav),
        tensorboard_enabled=tensorboard_enabled,
        tensorboard_log_dir=tensorboard_log_dir,
    )
