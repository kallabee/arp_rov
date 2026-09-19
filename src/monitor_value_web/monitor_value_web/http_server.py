from __future__ import annotations

import json
import mimetypes
import posixpath
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import unquote, urlparse

from monitor_value_web.config import load_web_config, public_config


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

    def _mtime(self) -> float:
        try:
            return float(self.cfg_path.stat().st_mtime)
        except OSError:
            return -1.0

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
            self.public_cfg = public_config(self.cfg)
            self._cfg_mtime = mtime

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
                "config": self.public_cfg,
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
        self._static(path)

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
