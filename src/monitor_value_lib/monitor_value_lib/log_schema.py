from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True, order=True)
class SchemaVersion:
    major: int
    minor: int
    patch: int = 0

    def __post_init__(self) -> None:
        if self.major < 0 or self.minor < 0 or self.patch < 0:
            raise ValueError("schema version components must be non-negative")

    def __str__(self) -> str:  # noqa: D105
        return f"{self.major}.{self.minor}.{self.patch}"

    @classmethod
    def parse(cls, s: str) -> "SchemaVersion":
        parts = [p.strip() for p in str(s).split(".")]
        if len(parts) != 3:
            raise ValueError(f"invalid schema version: {s!r}")
        return cls(int(parts[0]), int(parts[1]), int(parts[2]))


def auto_minor_from_columns(columns: Sequence[str]) -> int:
    """
    Auto-bump strategy (minor): derive from number of columns.

    This guarantees monotonic increase when columns are only appended over time,
    which matches the common "parameter count grows" logging evolution.
    """
    return int(len(list(columns)))


def schema_version_for_columns(*, columns: Sequence[str], major: int = 1, patch: int = 0) -> SchemaVersion:
    return SchemaVersion(major=int(major), minor=auto_minor_from_columns(columns), patch=int(patch))
