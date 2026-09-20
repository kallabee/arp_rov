from __future__ import annotations

import json
import mimetypes
import posixpath
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Optional
from urllib.parse import unquote, urlparse

from monitor_value_web.camera import CameraController, CameraError
from monitor_value_web.command import (
    EchoFilter,
    default_command,
    merge_lights,
    parse_hand,
    parse_lights,
    parse_twist,
    zero_hand,
    zero_twist,
)
from monitor_value_web.config import load_web_config, public_config
from monitor_value_web.record import RecordingJanitor

PublishFn = Callable[[dict[str, dict[str, float]]], None]


def _status_from_age(age: Optional[float], stale_sec: float) -> str:
    if age is None:
        return "never"
    if age > stale_sec:
        return "stale"
    return "live"


class DashboardState:
    def __init__(self, cfg: dict[str, Any], cfg_path: Path):
        self.cfg_path = Path(cfg_path)
        self.cfg = cfg
        self.public_cfg = public_config(cfg)
        self._cfg_mtime = self._mtime()
        self._lock = threading.Lock()
        self._monitor: Optional[dict[str, Any]] = None
        self._attitude: Optional[dict[str, Any]] = None
        self._monitor_mono: Optional[float] = None
        self._attitude_mono: Optional[float] = None
        self._attitude_src = "none"
        self._cond = threading.Condition(self._lock)
        self._tick = 0
        self.started_mono = time.monotonic()
        self._command = default_command(cfg.get("lights") or [])
        self._echo = EchoFilter(float(cfg.get("command", {}).get("echo_window_sec", 1.5)))
        self._active_until = {"twist": 0.0, "hand": 0.0, "lights": 0.0}
        self.publish_command: Optional[PublishFn] = None
        self._cameras = CameraController(cfg)
        self._camera_cache: Optional[dict[str, Any]] = None
        self._camera_cache_mono = 0.0
        self._camera_lock = threading.Lock()
        self._janitor = RecordingJanitor((cfg.get("cameras") or {}).get("record") or {})
        self._janitor.start()

    def _mtime(self) -> float:
        try:
            return float(self.cfg_path.stat().st_mtime)
        except OSError:
            return -1.0

    def _light_names(self) -> list[str]:
        return list(self.cfg.get("lights") or [])

    def refresh_config(self) -> None:
        mtime = self._mtime()
        if mtime == self._cfg_mtime:
            return
        try:
            loaded = load_web_config(self.cfg_path)
        except Exception:
            return
        with self._lock:
            self.cfg["stale_sec"] = loaded["stale_sec"]
            self.cfg["temperatures"] = loaded["temperatures"]
            self.cfg["temperature_gauge"] = loaded["temperature_gauge"]
            self.cfg["leaks"] = loaded["leaks"]
            self.cfg["gauges"] = loaded["gauges"]
            self.cfg["topics"] = loaded["topics"]
            self.cfg["lights"] = loaded["lights"]
            self.cfg["command"] = loaded["command"]
            self.cfg["cameras"] = loaded["cameras"]
            self.public_cfg = public_config(self.cfg)
            self._cameras = CameraController(self.cfg)
            self._camera_cache = None
            self._janitor.update((loaded.get("cameras") or {}).get("record") or {})
            self._cfg_mtime = mtime
            names = list(loaded["lights"])
            lights = self._command.get("lights") or {}
            self._command["lights"] = {n: float(lights.get(n, 0.0)) for n in names}

    def set_monitor(self, payload: dict[str, Any]) -> None:
        with self._cond:
            self._monitor = payload
            self._monitor_mono = time.monotonic()
            if self._attitude_src != "snapshot":
                att = {
                    "seq": payload.get("seq"),
                    "stamp": payload.get("stamp"),
                    "ori_x": payload.get("ori_x"),
                    "ori_y": payload.get("ori_y"),
                    "ori_z": payload.get("ori_z"),
                    "ori_w": payload.get("ori_w"),
                    "roll_deg": payload.get("roll_deg"),
                    "pitch_deg": payload.get("pitch_deg"),
                    "yaw_deg": payload.get("yaw_deg"),
                    "source": "monitor",
                }
                self._attitude = att
                self._attitude_mono = self._monitor_mono
                self._attitude_src = "monitor"
            self._tick += 1
            self._cond.notify_all()

    def set_attitude(self, payload: dict[str, Any]) -> None:
        with self._cond:
            payload = dict(payload)
            payload["source"] = "snapshot"
            self._attitude = payload
            self._attitude_mono = time.monotonic()
            self._attitude_src = "snapshot"
            self._tick += 1
            self._cond.notify_all()

    def cameras_snapshot(self, *, force: bool = False) -> dict[str, Any]:
        now = time.monotonic()
        with self._camera_lock:
            if (
                not force
                and self._camera_cache is not None
                and now - self._camera_cache_mono < 1.5
            ):
                return dict(self._camera_cache)
            try:
                snap = self._cameras.snapshot()
            except CameraError as exc:
                snap = {**self._cameras.meta(), "state": [], "error": str(exc)}
            self._camera_cache = snap
            self._camera_cache_mono = now
            return dict(snap)

    def apply_camera_command(self, cam_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        with self._camera_lock:
            result = self._cameras.apply(cam_id, payload)
            self._camera_cache = None
        with self._cond:
            self._tick += 1
            self._cond.notify_all()
        return result

    def apply_http_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        names = self._light_names()
        groups: dict[str, dict[str, float]] = {}
        now = time.monotonic()
        hold = 0.4
        with self._cond:
            if payload.get("estop"):
                groups["twist"] = zero_twist()
                groups["hand"] = zero_hand()
            else:
                if "twist" in payload:
                    groups["twist"] = parse_twist(payload.get("twist"))
                if "hand" in payload:
                    groups["hand"] = parse_hand(payload.get("hand"))
            if "lights" in payload:
                groups["lights"] = parse_lights(payload.get("lights"), names)
            if not groups:
                return dict(self._command)
            for group, data in groups.items():
                self._command[group] = data
                self._active_until[group] = now + hold
                self._echo.note(group, data)
            self._command["origin"] = "http"
            self._command["seq"] = int(self._command.get("seq") or 0) + 1
            snapshot = {
                "twist": dict(self._command["twist"]),
                "hand": dict(self._command["hand"]),
                "lights": dict(self._command["lights"]),
                "origin": "http",
                "seq": self._command["seq"],
            }
            self._tick += 1
            self._cond.notify_all()
        if self.publish_command is not None:
            self.publish_command(groups)
        return snapshot

    def apply_remote_command(self, group: str, data: dict[str, float]) -> bool:
        now = time.monotonic()
        with self._cond:
            if now < self._active_until.get(group, 0.0):
                return False
            if self._echo.is_echo(group, data):
                return False
            if group == "lights":
                self._command["lights"] = merge_lights(
                    self._command.get("lights") or {}, data, self._light_names()
                )
            else:
                self._command[group] = dict(data)
            self._command["origin"] = "ros"
            self._command["seq"] = int(self._command.get("seq") or 0) + 1
            self._tick += 1
            self._cond.notify_all()
            return True
        return False

    def stop(self) -> None:
        self._janitor.stop()

    def snapshot(self) -> dict[str, Any]:
        self.refresh_config()
        now = time.monotonic()
        stale_sec = float(self.cfg["stale_sec"])
        with self._lock:
            mon_age = None if self._monitor_mono is None else now - self._monitor_mono
            att_age = None if self._attitude_mono is None else now - self._attitude_mono
            publisher = _status_from_age(mon_age, stale_sec)
            imu = _status_from_age(att_age, stale_sec)
            leak = False
            if self._monitor is not None:
                leak = bool(self._monitor.get("water_ch0_detected") or self._monitor.get("water_ch1_detected"))
            return {
                "monitor": self._monitor,
                "attitude": self._attitude,
                "command": {
                    "twist": dict(self._command["twist"]),
                    "hand": dict(self._command["hand"]),
                    "lights": dict(self._command["lights"]),
                    "origin": self._command.get("origin") or "none",
                    "seq": int(self._command.get("seq") or 0),
                },
                "config": self.public_cfg,
                "cameras": dict(self._camera_cache) if self._camera_cache else self._cameras.meta(),
                "health": {
                    "ok": publisher == "live",
                    "publisher": publisher,
                    "imu": imu,
                    "imu_source": self._attitude_src,
                    "age_monitor_sec": mon_age,
                    "age_imu_sec": att_age,
                    "uptime_sec": now - self.started_mono,
                    "leak": leak,
                    "config_path": str(self.cfg_path),
                },
            }

    def wait_tick(self, last_tick: int, timeout: float) -> int:
        with self._cond:
            if self._tick != last_tick:
                return self._tick
            self._cond.wait(timeout=timeout)
            return self._tick


class DashboardHandler(BaseHTTPRequestHandler):
    state: DashboardState
    static_root: Path

    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send_json(self, code: int, payload: Any) -> None:
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/health":
            self._send_json(200, self.state.snapshot()["health"])
            return
        if path == "/api/latest":
            self._send_json(200, self.state.snapshot())
            return
        if path == "/api/config":
            self.state.refresh_config()
            self._send_json(200, self.state.public_cfg)
            return
        if path == "/api/stream":
            self._stream()
            return
        if path == "/api/cameras":
            try:
                self._send_json(200, self.state.cameras_snapshot())
            except CameraError as exc:
                self._send_json(503, {"error": str(exc)})
            return
        self._static(path)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_json(400, {"error": "invalid content-length"})
            return
        if length < 0 or length > 100_000:
            self._send_json(413, {"error": "payload too large"})
            return
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"error": "invalid json"})
            return
        if not isinstance(payload, dict):
            self._send_json(400, {"error": "json object required"})
            return
        if path == "/api/command":
            try:
                result = self.state.apply_http_command(payload)
            except (TypeError, ValueError) as exc:
                self._send_json(400, {"error": str(exc)})
                return
            self._send_json(200, result)
            return
        if path.startswith("/api/cameras/"):
            cam_id = unquote(path[len("/api/cameras/") :]).strip("/")
            if not cam_id or "/" in cam_id:
                self._send_json(404, {"error": "unknown camera"})
                return
            try:
                result = self.state.apply_camera_command(cam_id, payload)
            except CameraError as exc:
                self._send_json(400, {"error": str(exc)})
                return
            self._send_json(200, result)
            return
        self.send_error(404)

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        last = -1
        try:
            while True:
                last = self.state.wait_tick(last, timeout=1.0)
                payload = json.dumps(self.state.snapshot(), allow_nan=False)
                chunk = f"event: state\ndata: {payload}\n\n".encode("utf-8")
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _static(self, path: str) -> None:
        rel = posixpath.normpath(unquote(path).lstrip("/"))
        if rel in ("", ".", "/"):
            rel = "index.html"
        if rel == ".." or rel.startswith("../"):
            self.send_error(403)
            return
        root = self.static_root
        target = root / rel
        try:
            target.relative_to(root)
        except ValueError:
            self.send_error(403)
            return
        if target.is_dir():
            target = target / "index.html"
        if not target.is_file():
            suffix = Path(rel).suffix.lower()
            if suffix and suffix != ".html":
                self.send_error(404)
                return
            index = root / "index.html"
            if index.is_file() and not rel.startswith("api/"):
                target = index
            else:
                self.send_error(404)
                return
        ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def start_http_server(
    state: DashboardState,
    static_root: Path,
    host: str,
    port: int,
) -> ThreadingHTTPServer:
    handler = type(
        "BoundDashboardHandler",
        (DashboardHandler,),
        {"state": state, "static_root": static_root},
    )
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
