from pathlib import Path

from rpi_camera_ctrl.record import (
    durations_equal,
    mediamtx_record_patch,
    normalize_record_config,
    parse_bytes,
    parse_duration_sec,
    reap_recordings,
    record_patch_needed,
    recording_status,
)


def test_parse_duration_go_style():
    assert parse_duration_sec("10m") == 600
    assert parse_duration_sec("1h0m0s") == 3600
    assert parse_duration_sec("1s") == 1
    assert parse_duration_sec("") == 0
    assert durations_equal("0s", "")
    assert durations_equal("10m", "10m0s")


def test_parse_bytes_si_and_binary():
    assert parse_bytes("10G") == 10_000_000_000
    assert parse_bytes("10GiB") == 10 * 1024**3
    assert parse_bytes(2048) == 2048


def test_recording_status_active_only_when_ready():
    assert recording_status({"record": True}, {"ready": True}) == {
        "enabled": True,
        "active": True,
    }
    assert recording_status({"record": True}, {"ready": False})["active"] is False
    assert recording_status({"record": False}, {"ready": True})["active"] is False


def test_mediamtx_patch_and_needed():
    cfg = normalize_record_config(
        {"enabled": True, "dir": "/home/arp/mediamtx/recordings", "segment": "10m", "part": "1s"}
    )
    desired = mediamtx_record_patch(cfg)
    assert desired["record"] is True
    assert desired["recordFormat"] == "fmp4"
    assert desired["recordPath"].endswith("/%path/%path_%Y-%m-%d_%H-%M-%S-%f")
    assert desired["recordDeleteAfter"] == "0s"
    current = dict(desired)
    current["recordSegmentDuration"] = "10m0s"
    assert record_patch_needed(current, desired) is False
    current["record"] = False
    assert record_patch_needed(current, desired) is True


def test_reap_deletes_oldest_until_min_free(tmp_path, monkeypatch):
    root = tmp_path / "recordings"
    cam = root / "cam0"
    cam.mkdir(parents=True)
    oldest = cam / "a.mp4"
    middle = cam / "b.mp4"
    newest = cam / "c.mp4"
    for path, age, size in (
        (oldest, 300, 4000),
        (middle, 200, 4000),
        (newest, 5, 4000),
    ):
        path.write_bytes(b"x" * size)
        path.touch()
    # Force mtimes independently of write order.
    import os

    now = 1_700_000_000.0
    os.utime(oldest, (now - 300, now - 300))
    os.utime(middle, (now - 200, now - 200))
    os.utime(newest, (now - 5, now - 5))

    state = {"free": 1000}

    class Usage:
        def __init__(self, free):
            self.free = free
            self.total = 20_000
            self.used = self.total - free

    def fake_usage(_path):
        return Usage(state["free"])

    real_unlink = Path.unlink

    def tracking_unlink(self, *args, **kwargs):
        size = self.stat().st_size
        real_unlink(self, *args, **kwargs)
        state["free"] += size

    monkeypatch.setattr("rpi_camera_ctrl.record.shutil.disk_usage", fake_usage)
    monkeypatch.setattr(Path, "unlink", tracking_unlink)

    stats = reap_recordings(
        {
            "dir": str(root),
            "min_free": 12000,
            "protect_recent_sec": 20,
        },
        now=now,
    )
    assert oldest.exists() is False
    assert middle.exists() is False
    assert newest.exists() is True
    assert stats["skipped_recent"] == 1
    assert stats["ok"] is False
    assert stats["free_bytes"] == 9000
