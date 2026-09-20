from __future__ import annotations

import json
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from rpi_camera_ctrl.camera import CameraController, CameraError
from rpi_camera_ctrl.config import load_camera_config
from rpi_camera_ctrl.record import RecordingJanitor
from rpi_camera_ctrl.ros_codec import snapshot_to_msg, state_to_msg
from rpi_camera_interfaces.msg import RpiCameraSnapshot
from rpi_camera_interfaces.srv import GetRpiCameras, SetRpiCamera


def _cfg_default_path() -> Path:
    src = Path(__file__).resolve().parents[1] / "config" / "rpi_camera_ctrl.yaml"
    if src.is_file():
        return src
    env = os.environ.get("RPI_CAMERA_CTRL_CONFIG")
    if env:
        p = Path(env)
        if p.is_file():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("rpi_camera_ctrl"))
        cand = share / "config" / "rpi_camera_ctrl.yaml"
        if cand.is_file():
            return cand
    except Exception:
        pass
    return src


class RpiCameraCtrl(Node):
    def __init__(self) -> None:
        super().__init__("rpi_camera_ctrl")
        cfg_path = _cfg_default_path()
        cfg = load_camera_config(cfg_path)
        self._cfg_path = cfg_path
        self._cfg = cfg
        self._controller = CameraController({"cameras": cfg["cameras"]})
        self._janitor = RecordingJanitor((cfg.get("cameras") or {}).get("record") or {})
        self._janitor.start()
        ros = cfg["ros"]
        self._snap_pub = self.create_publisher(RpiCameraSnapshot, ros["snapshot_topic"], 10)
        self.create_service(GetRpiCameras, ros["get_service"], self._on_get)
        self.create_service(SetRpiCamera, ros["set_service"], self._on_set)
        hz = max(0.2, float(ros["publish_hz"]))
        self.create_timer(1.0 / hz, self._publish_snapshot)
        self.get_logger().info(f"config: {cfg_path}")
        self.get_logger().info(
            f"backend={cfg['cameras'].get('backend')} "
            f"snapshot={ros['snapshot_topic']} "
            f"get={ros['get_service']} set={ros['set_service']}"
        )

    def _take_snapshot(self) -> dict:
        try:
            return self._controller.snapshot()
        except CameraError as exc:
            return {**self._controller.meta(), "state": [], "error": str(exc)}

    def _publish_snapshot(self) -> None:
        snap = self._take_snapshot()
        self._snap_pub.publish(snapshot_to_msg(snap))

    def _on_get(
        self, _request: GetRpiCameras.Request, response: GetRpiCameras.Response
    ) -> GetRpiCameras.Response:
        snap = self._take_snapshot()
        response.ok = not bool(snap.get("error"))
        response.error = str(snap.get("error") or "")
        response.snapshot = snapshot_to_msg(snap)
        return response

    def _on_set(
        self, request: SetRpiCamera.Request, response: SetRpiCamera.Response
    ) -> SetRpiCamera.Response:
        cam_id = str(request.camera_id or "").strip()
        try:
            payload = json.loads(request.request_json or "{}")
        except json.JSONDecodeError as exc:
            response.ok = False
            response.error = f"invalid request_json: {exc}"
            return response
        if not isinstance(payload, dict):
            response.ok = False
            response.error = "request_json must be an object"
            return response
        try:
            result = self._controller.apply(cam_id, payload)
        except CameraError as exc:
            response.ok = False
            response.error = str(exc)
            return response
        response.ok = True
        response.error = ""
        response.restart = bool(result.get("restart"))
        response.ready = bool(result.get("ready"))
        response.result_json = json.dumps(result, allow_nan=False)
        response.state = state_to_msg(result)
        self._snap_pub.publish(snapshot_to_msg(self._take_snapshot()))
        return response

    def destroy_node(self):
        try:
            self._janitor.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RpiCameraCtrl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
