# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path

import yaml

from monitor_value_lib.config import load_config
from monitor_value_lib.logger.tensorboard_logger import TensorboardLogger
from monitor_value_lib.values import MonitorValues


def test_tensorboard_enabled_by_default(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text(
        yaml.dump(
            {
                "log_dir": str(tmp_path / "log"),
                "log_rotate_seconds": 1800,
                "sample_interval_seconds": 1.0,
                "sensors": {"rpi": {"mode": "disabled"}},
            }
        ),
        encoding="utf-8",
    )
    cfg = load_config(p)
    assert cfg.tensorboard_enabled is True
    assert cfg.tensorboard_log_dir == Path(tmp_path / "log" / "tb")


def test_tensorboard_can_disable(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text(
        yaml.dump(
            {
                "log_dir": str(tmp_path / "log"),
                "tensorboard_enabled": False,
                "sensors": {"rpi": {"mode": "disabled"}},
            }
        ),
        encoding="utf-8",
    )
    cfg = load_config(p)
    assert cfg.tensorboard_enabled is False


def test_tensorboard_logger_writes_events(tmp_path: Path) -> None:
    tb = TensorboardLogger(log_dir=tmp_path / "tb")
    v0 = MonitorValues(seq=27000, rpi_cpu_temp_c=42.5, current_a=1.2)
    v1 = MonitorValues(seq=0, rpi_cpu_temp_c=43.0, current_a=1.3)  # non-monotonic msg seq
    out0 = tb.append(v0)
    out1 = tb.append(v1)
    tb.close()
    assert tb.disabled_reason is None
    assert out0 is not None and out1 == out0
    assert out0.is_dir()
    events = list(out0.glob("events.out.tfevents.*"))
    assert events, f"no event files under {out0}"

    # Steps must be logger-local 0,1 even if message seq jumps.
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError:
        return
    ea = EventAccumulator(str(out0), size_guidance={"scalars": 0})
    ea.Reload()
    tag = "rpi/cpu_temp_c"
    assert tag in ea.Tags().get("scalars", [])
    steps = [e.step for e in ea.Scalars(tag)]
    assert steps == [0, 1], steps
