"""On-disk camera recording helpers (MediaMTX fMP4 + free-space janitor)."""

from __future__ import annotations

import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Optional


_DURATION_RE = re.compile(r"(\d+(?:\.\d+)?)(ns|us|µs|ms|s|m|h|d)")
_SIZE_RE = re.compile(r"^(\d+(?:\.\d+)?)\s*([kmgtpe]i?b?)?$", re.I)
_DURATION_KEYS = frozenset(
    {"recordPartDuration", "recordSegmentDuration", "recordDeleteAfter"}
)

DEFAULT_RECORD = {
    "enabled": True,
    "dir": "/home/arp/mediamtx/recordings",
    "segment": "10m",
    "part": "1s",
    "min_free": "10G",
    "check_sec": 15.0,
    "protect_recent_sec": 20.0,
}


def parse_duration_sec(value: Any) -> float:
    """Parse Go-style durations (``10m``, ``1h0m0s``) or a plain second count."""
    if isinstance(value, (int, float)):
        if float(value) < 0:
            raise ValueError("duration must be >= 0")
        return float(value)
    raw = str(value or "").strip().lower().replace("µs", "us")
    if not raw:
        return 0.0
    try:
        return float(raw)
    except ValueError:
        pass
    total = 0.0
    pos = 0
    units = {
        "ns": 1e-9,
        "us": 1e-6,
        "ms": 1e-3,
        "s": 1.0,
        "m": 60.0,
        "h": 3600.0,
        "d": 86400.0,
    }
    for match in _DURATION_RE.finditer(raw):
        if match.start() != pos:
            raise ValueError(f"invalid duration: {value}")
        total += float(match.group(1)) * units[match.group(2)]
        pos = match.end()
    if pos == 0 or pos != len(raw):
        raise ValueError(f"invalid duration: {value}")
    return total


def durations_equal(left: Any, right: Any) -> bool:
    try:
        return abs(parse_duration_sec(left) - parse_duration_sec(right)) < 0.001
    except (TypeError, ValueError):
        return str(left).strip() == str(right).strip()


def parse_bytes(value: Any) -> int:
    """Parse a byte count. ``10G`` / ``10GB`` are SI; ``10GiB`` is binary."""
    if isinstance(value, bool):
        raise ValueError("byte size must not be a boolean")
    if isinstance(value, (int, float)):
        if value < 0:
            raise ValueError("byte size must be >= 0")
        return int(value)
    raw = str(value or "").strip().replace(" ", "")
    if not raw:
        raise ValueError("empty byte size")
    match = _SIZE_RE.match(raw)
    if not match:
        raise ValueError(f"invalid byte size: {value}")
    amount = float(match.group(1))
    suffix = (match.group(2) or "").upper()
    table = {
        "": 1,
        "B": 1,
        "K": 1000,
        "KB": 1000,
        "M": 1_000_000,
        "MB": 1_000_000,
        "G": 1_000_000_000,
        "GB": 1_000_000_000,
        "T": 1_000_000_000_000,
        "TB": 1_000_000_000_000,
        "KI": 1024,
        "KIB": 1024,
        "MI": 1024**2,
        "MIB": 1024**2,
        "GI": 1024**3,
        "GIB": 1024**3,
        "TI": 1024**4,
        "TIB": 1024**4,
    }
    if suffix not in table:
        raise ValueError(f"invalid byte size: {value}")
    return int(amount * table[suffix])


def normalize_record_config(raw: Any) -> dict[str, Any]:
    src = raw if isinstance(raw, Mapping) else {}
    enabled = bool(src.get("enabled", DEFAULT_RECORD["enabled"]))
    directory = str(src.get("dir") or DEFAULT_RECORD["dir"]).strip() or str(
        DEFAULT_RECORD["dir"]
    )
    segment = str(src.get("segment") or DEFAULT_RECORD["segment"]).strip()
    part = str(src.get("part") or DEFAULT_RECORD["part"]).strip()
    min_free_raw = src.get("min_free", DEFAULT_RECORD["min_free"])
    parse_duration_sec(segment)
    parse_duration_sec(part)
    min_free_bytes = parse_bytes(min_free_raw)
    check_sec = float(src.get("check_sec", DEFAULT_RECORD["check_sec"]))
    protect_recent_sec = float(
        src.get("protect_recent_sec", DEFAULT_RECORD["protect_recent_sec"])
    )
    if check_sec < 1:
        check_sec = 1.0
    if protect_recent_sec < 1:
        protect_recent_sec = 1.0
    return {
        "enabled": enabled,
        "dir": directory,
        "segment": segment,
        "part": part,
        "min_free": str(min_free_raw),
        "min_free_bytes": min_free_bytes,
        "check_sec": check_sec,
        "protect_recent_sec": protect_recent_sec,
    }


def record_path_template(record_cfg: Mapping[str, Any]) -> str:
    root = str(record_cfg.get("dir") or DEFAULT_RECORD["dir"]).rstrip("/")
    return f"{root}/%path/%path_%Y-%m-%d_%H-%M-%S-%f"


def mediamtx_record_patch(record_cfg: Mapping[str, Any]) -> dict[str, Any]:
    cfg = normalize_record_config(record_cfg)
    if not cfg["enabled"]:
        return {"record": False}
    return {
        "record": True,
        "recordPath": record_path_template(cfg),
        "recordFormat": "fmp4",
        "recordPartDuration": cfg["part"],
        "recordMaxPartSize": "50M",
        "recordSegmentDuration": cfg["segment"],
        "recordDeleteAfter": "0s",
    }


def record_patch_needed(current: Mapping[str, Any], desired: Mapping[str, Any]) -> bool:
    for key, want in desired.items():
        have = current.get(key)
        if key == "record":
            if bool(have) != bool(want):
                return True
            continue
        if key in _DURATION_KEYS:
            if not durations_equal(have, want):
                return True
            continue
        if str(have or "") != str(want):
            return True
    return False


def recording_status(conf: Mapping[str, Any], runtime: Mapping[str, Any]) -> dict[str, Any]:
    enabled = bool(conf.get("record"))
    ready = bool(runtime.get("ready") or runtime.get("online"))
    return {"enabled": enabled, "active": bool(enabled and ready)}


def storage_snapshot(record_cfg: Mapping[str, Any]) -> dict[str, Any]:
    cfg = normalize_record_config(record_cfg)
    path = Path(cfg["dir"])
    min_free = int(cfg["min_free_bytes"])
    try:
        target = path if path.exists() else path.parent
        usage = shutil.disk_usage(target)
        free = int(usage.free)
        total = int(usage.total)
    except OSError:
        free = 0
        total = 0
    return {
        "path": str(path),
        "free_bytes": free,
        "total_bytes": total,
        "min_free_bytes": min_free,
        "ok": free >= min_free if min_free > 0 else True,
    }


def _assert_safe_record_dir(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved == Path("/") or len(resolved.parts) < 3:
        raise ValueError(f"record dir is too broad: {resolved}")
    return resolved


def iter_segments(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    files = [
        p
        for p in root.rglob("*")
        if p.is_file() and p.suffix.lower() in {".mp4", ".ts"}
    ]
    files.sort(key=lambda p: (p.stat().st_mtime, p.name))
    return files


def reap_recordings(
    record_cfg: Mapping[str, Any],
    *,
    now: Optional[float] = None,
) -> dict[str, Any]:
    """Delete oldest segments until remaining free space meets ``min_free``."""
    cfg = normalize_record_config(record_cfg)
    root = _assert_safe_record_dir(Path(cfg["dir"]))
    min_free = int(cfg["min_free_bytes"])
    protect = float(cfg["protect_recent_sec"])
    stamp = time.time() if now is None else float(now)
    stats = storage_snapshot(cfg)
    stats["deleted"] = []
    stats["freed_bytes"] = 0
    stats["skipped_recent"] = 0
    if min_free <= 0 or stats["free_bytes"] >= min_free:
        return stats
    if not root.is_dir():
        return stats
    for path in iter_segments(root):
        if stats["free_bytes"] >= min_free:
            break
        try:
            info = path.stat()
        except OSError:
            continue
        if stamp - info.st_mtime < protect:
            stats["skipped_recent"] += 1
            continue
        try:
            path.unlink()
        except PermissionError:
            try:
                subprocess.run(
                    ["sudo", "-n", "rm", "-f", "--", str(path)],
                    check=False,
                    timeout=5,
                )
                if path.exists():
                    continue
            except Exception:
                continue
        except OSError:
            continue
        stats["deleted"].append(str(path))
        stats["freed_bytes"] += int(info.st_size)
        try:
            usage = shutil.disk_usage(root)
            stats["free_bytes"] = int(usage.free)
            stats["total_bytes"] = int(usage.total)
        except OSError:
            stats["free_bytes"] += int(info.st_size)
    stats["ok"] = stats["free_bytes"] >= min_free if min_free > 0 else True
    return stats


class RecordingJanitor:
    """Background loop that keeps ``min_free`` bytes available under ``dir``."""

    def __init__(self, record_cfg: Any = None):
        self._lock = threading.Lock()
        self._cfg = normalize_record_config(record_cfg or {})
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_stats: dict[str, Any] = {}

    def update(self, record_cfg: Any) -> None:
        with self._lock:
            self._cfg = normalize_record_config(record_cfg or {})

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="record-janitor", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def run_once(self) -> dict[str, Any]:
        with self._lock:
            cfg = dict(self._cfg)
        try:
            stats = reap_recordings(cfg)
        except Exception as exc:
            stats = {"ok": False, "error": str(exc), "deleted": []}
        with self._lock:
            self.last_stats = stats
        return stats

    def _loop(self) -> None:
        self.run_once()
        while True:
            with self._lock:
                interval = float(self._cfg["check_sec"])
            if self._stop.wait(interval):
                return
            self.run_once()


def recording_public_meta(record_cfg: Mapping[str, Any]) -> dict[str, Any]:
    cfg = normalize_record_config(record_cfg)
    return {
        "enabled": cfg["enabled"],
        "dir": cfg["dir"],
        "segment": cfg["segment"],
        "part": cfg["part"],
        "min_free_bytes": cfg["min_free_bytes"],
    }
