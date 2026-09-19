from __future__ import annotations

import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from monitor_value_lib.csv_schema import csv_header, values_to_csv_row
from monitor_value_lib.values import MonitorValues


def _start_filename(ts: datetime) -> str:
    # Program start time; UTC so filename is stable across machines
    return ts.astimezone(timezone.utc).strftime("%Y%m%d_%H%M%S")


@dataclass
class RotatingCsvLogger:
    log_dir: Path
    rotate_seconds: float
    flush_each_row: bool = True

    _file: Optional[object] = None
    _file_start_monotonic: float = 0.0
    _file_index: int = 0
    _base_ts: Optional[datetime] = None
    _path: Optional[Path] = None
    _wrote_header: bool = False

    def _ensure_open(self) -> None:
        if self._file is not None:
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._base_ts = datetime.now(timezone.utc)
        self._file_index = 0
        self._open_new_file()

    def _open_new_file(self) -> None:
        assert self._base_ts is not None
        if self._file is not None:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            finally:
                self._file.close()
        base = _start_filename(self._base_ts)
        suffix = "" if self._file_index == 0 else f"_{self._file_index}"
        self._path = self.log_dir / f"{base}{suffix}.csv"
        self._file = self._path.open("a", encoding="utf-8", buffering=1)
        self._file_start_monotonic = time.monotonic()
        self._wrote_header = False

    def _maybe_rotate(self) -> None:
        if self.rotate_seconds <= 0:
            return
        if self._file is None:
            return
        age = time.monotonic() - self._file_start_monotonic
        if age < self.rotate_seconds:
            return
        self._file_index += 1
        self._open_new_file()

    def append(self, values: MonitorValues) -> Path:
        self._ensure_open()
        self._maybe_rotate()
        assert self._file is not None
        assert self._path is not None

        if not self._wrote_header:
            self._file.write(",".join(csv_header()) + "\n")
            self._wrote_header = True

        self._file.write(",".join(values_to_csv_row(values)) + "\n")
        if self.flush_each_row:
            self._file.flush()
            os.fsync(self._file.fileno())
        return self._path

    def close(self) -> None:
        if self._file is None:
            return
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
        finally:
            self._file.close()
            self._file = None
