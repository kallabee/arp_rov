"""Minimal TensorBoard event-file writer (scalars only, no torch/tensorboardX).

Writes ``events.out.tfevents.*`` files that ``tensorboard --logdir ...`` can open
locally, remotely, or offline after copying the directory.
"""

from __future__ import annotations

import os
import struct
import time
from pathlib import Path
from typing import Optional


# CRC-32C (Castagnoli) — required by TensorBoard record framing.
_CRC32C_TABLE: list[int] | None = None


def _crc32c_table() -> list[int]:
    global _CRC32C_TABLE
    if _CRC32C_TABLE is not None:
        return _CRC32C_TABLE
    table: list[int] = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if (crc & 1) else (crc >> 1)
        table.append(crc)
    _CRC32C_TABLE = table
    return table


def _crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    table = _crc32c_table()
    for b in data:
        crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def _masked_crc32c(data: bytes) -> int:
    crc = _crc32c(data)
    return (((crc >> 15) | (crc << 17)) + 0xA282EAD8) & 0xFFFFFFFF


def _encode_varint(value: int) -> bytes:
    out = bytearray()
    v = int(value)
    while True:
        bits = v & 0x7F
        v >>= 7
        if v:
            out.append(bits | 0x80)
        else:
            out.append(bits)
            break
    return bytes(out)


def _tag_wire(field_number: int, wire_type: int) -> bytes:
    return _encode_varint((field_number << 3) | wire_type)


def _encode_string_field(field_number: int, s: str) -> bytes:
    b = s.encode("utf-8")
    return _tag_wire(field_number, 2) + _encode_varint(len(b)) + b


def _encode_bytes_field(field_number: int, b: bytes) -> bytes:
    return _tag_wire(field_number, 2) + _encode_varint(len(b)) + b


def _encode_float_field(field_number: int, value: float) -> bytes:
    return _tag_wire(field_number, 5) + struct.pack("<f", float(value))


def _encode_double_field(field_number: int, value: float) -> bytes:
    return _tag_wire(field_number, 1) + struct.pack("<d", float(value))


def _encode_int64_field(field_number: int, value: int) -> bytes:
    return _tag_wire(field_number, 0) + _encode_varint(int(value))


def _encode_summary_value(tag: str, simple_value: float) -> bytes:
    # summary.Value { tag=1, simple_value=2 }
    inner = _encode_string_field(1, tag) + _encode_float_field(2, simple_value)
    return _encode_bytes_field(1, inner)  # Summary.value = 1


def _encode_event_summary(wall_time: float, step: int, tag: str, value: float) -> bytes:
    # Event { wall_time=1, step=2, summary=5 }
    summary = _encode_summary_value(tag, value)
    return (
        _encode_double_field(1, wall_time)
        + _encode_int64_field(2, step)
        + _encode_bytes_field(5, summary)
    )


def _encode_event_file_version(wall_time: float) -> bytes:
    # Event { wall_time=1, file_version=3 }
    return _encode_double_field(1, wall_time) + _encode_string_field(3, "brain.Event:2")


def _write_record(fp, data: bytes) -> None:
    header = struct.pack("<Q", len(data))
    fp.write(header)
    fp.write(struct.pack("<I", _masked_crc32c(header)))
    fp.write(data)
    fp.write(struct.pack("<I", _masked_crc32c(data)))


class PureSummaryWriter:
    """Tiny SummaryWriter-compatible API for scalar logging."""

    def __init__(self, log_dir: str | Path):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        # Match TF naming so TensorBoard picks the file up.
        fname = f"events.out.tfevents.{int(time.time())}.{os.getpid()}"
        self._path = self.log_dir / fname
        self._fp = self._path.open("wb")
        _write_record(self._fp, _encode_event_file_version(time.time()))
        self._fp.flush()

    def add_scalar(
        self,
        tag: str,
        scalar_value: float,
        global_step: Optional[int] = None,
        walltime: Optional[float] = None,
    ) -> None:
        step = 0 if global_step is None else int(global_step)
        wt = time.time() if walltime is None else float(walltime)
        _write_record(self._fp, _encode_event_summary(wt, step, tag, float(scalar_value)))

    def flush(self) -> None:
        self._fp.flush()
        try:
            os.fsync(self._fp.fileno())
        except OSError:
            pass

    def close(self) -> None:
        try:
            self.flush()
        finally:
            self._fp.close()
