"""Backward-compatible re-exports for camera pan/tilt helpers."""

from thruster_controller.cam_act_config import (
    CamActPTConfig,
    CamActPTZConfig,
    PanTiltAxisPTConfig,
    PanTiltAxisPTZConfig,
)
from thruster_controller.cam_act_controllers import (
    CamActControllerBase,
    CamActControllerPT,
    CamActControllerPTZ,
    ServoAngleRegulator,
)

__all__ = [
    "CamActControllerBase",
    "CamActControllerPT",
    "CamActControllerPTZ",
    "CamActPTConfig",
    "CamActPTZConfig",
    "PanTiltAxisPTConfig",
    "PanTiltAxisPTZConfig",
    "ServoAngleRegulator",
]
