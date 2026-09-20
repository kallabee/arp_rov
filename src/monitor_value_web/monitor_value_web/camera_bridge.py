"""ROS client bridge for rpi_camera_ctrl (HTTP thread → ROS executor)."""

from __future__ import annotations

import json
import queue
import threading
from typing import Any, Optional

from rpi_camera_interfaces.msg import RpiCameraSnapshot
from rpi_camera_interfaces.srv import GetRpiCameras, SetRpiCamera


class CameraBridgeError(RuntimeError):
    """Camera ROS call failed or timed out."""


class RosCameraBridge:
    """Serialize camera get/set onto the ROS executor via a work queue."""

    def __init__(self, *, timeout_sec: float = 12.0):
        self._q: queue.Queue = queue.Queue()
        self._timeout = float(timeout_sec)
        self._last: Optional[dict[str, Any]] = None
        self._lock = threading.Lock()
        self._node = None
        self.get_client = None
        self.set_client = None
        self._inflight: Optional[tuple[str, Any, queue.Queue, Any]] = None

    def attach(self, node, get_client, set_client) -> None:
        self._node = node
        self.get_client = get_client
        self.set_client = set_client

    def on_snapshot_msg(self, msg: RpiCameraSnapshot) -> None:
        try:
            data = json.loads(msg.snapshot_json or "{}")
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            self._last = data

    def meta(self) -> dict[str, Any]:
        with self._lock:
            if self._last is not None:
                snap = dict(self._last)
                for key in ("state", "storage", "error"):
                    snap.pop(key, None)
                return snap
        return {
            "backend": "unknown",
            "webrtc": "http://127.0.0.1:8889",
            "items": [],
            "zoom": [],
            "live": [],
            "restart": [],
            "record": {"enabled": False, "dir": "", "segment": "", "part": "", "min_free_bytes": 0},
        }

    def cached_snapshot(self) -> Optional[dict[str, Any]]:
        with self._lock:
            return dict(self._last) if self._last is not None else None

    def snapshot(self, *, force: bool = False) -> dict[str, Any]:
        if not force:
            cached = self.cached_snapshot()
            if cached is not None:
                return cached
        return self._request("get", None)

    def apply(self, cam_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self._request("set", (cam_id, payload))

    def _request(self, op: str, arg: Any) -> dict[str, Any]:
        fut: queue.Queue = queue.Queue(maxsize=1)
        self._q.put((op, arg, fut))
        try:
            ok, payload = fut.get(timeout=self._timeout)
        except queue.Empty as exc:
            raise CameraBridgeError("camera ROS call timed out") from exc
        if not ok:
            raise CameraBridgeError(str(payload))
        if not isinstance(payload, dict):
            raise CameraBridgeError("invalid camera response")
        return payload

    def pump(self) -> None:
        """Advance queued work. Call from a ROS timer on the node executor."""
        if self._inflight is not None:
            self._poll_inflight()
            return
        try:
            op, arg, fut = self._q.get_nowait()
        except queue.Empty:
            return
        try:
            if op == "get":
                client = self.get_client
                if client is None:
                    raise CameraBridgeError("camera get client not ready")
                if not client.service_is_ready():
                    cached = self.cached_snapshot()
                    if cached is not None:
                        fut.put((True, cached))
                        return
                    raise CameraBridgeError("rpi_camera get service unavailable")
                future = client.call_async(GetRpiCameras.Request())
            elif op == "set":
                client = self.set_client
                if client is None:
                    raise CameraBridgeError("camera set client not ready")
                if not client.service_is_ready():
                    raise CameraBridgeError("rpi_camera set service unavailable")
                cam_id, payload = arg
                req = SetRpiCamera.Request()
                req.camera_id = str(cam_id)
                req.request_json = json.dumps(payload, allow_nan=False)
                future = client.call_async(req)
            else:
                raise CameraBridgeError(f"unknown camera op: {op}")
        except Exception as exc:  # noqa: BLE001
            fut.put((False, str(exc)))
            return
        self._inflight = (op, arg, fut, future)
        self._poll_inflight()

    def _poll_inflight(self) -> None:
        if self._inflight is None:
            return
        op, _arg, fut, future = self._inflight
        if not future.done():
            return
        self._inflight = None
        try:
            resp = future.result()
            if resp is None:
                raise CameraBridgeError(f"camera {op} returned no response")
            if op == "get":
                data = json.loads(resp.snapshot.snapshot_json or "{}")
                if not isinstance(data, dict):
                    raise CameraBridgeError("invalid snapshot_json")
                if not resp.ok and resp.error and not data.get("state"):
                    raise CameraBridgeError(resp.error)
                with self._lock:
                    self._last = dict(data)
                fut.put((True, data))
            else:
                if not resp.ok:
                    raise CameraBridgeError(resp.error or "camera set failed")
                data = json.loads(resp.result_json or "{}")
                if not isinstance(data, dict):
                    raise CameraBridgeError("invalid result_json")
                with self._lock:
                    self._last = None
                fut.put((True, data))
        except Exception as exc:  # noqa: BLE001
            fut.put((False, str(exc)))
