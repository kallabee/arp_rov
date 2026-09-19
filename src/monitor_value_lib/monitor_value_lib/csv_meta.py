from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import IO, Optional


META_PREFIX = "#"


def _escape_meta_value(v: str) -> str:
    # Keep it simple: single-line key=value. Newlines are not supported.
    return v.replace("\n", "\\n")


def _unescape_meta_value(v: str) -> str:
    return v.replace("\\n", "\n")


def write_meta_header_line(fp: IO[str], key: str, value: str) -> None:
    fp.write(f"{META_PREFIX} {key}={_escape_meta_value(value)}\n")


def write_meta_header(fp: IO[str], meta: Mapping[str, str]) -> None:
    # Stable output helps diffs and downstream tooling.
    for k in sorted(meta.keys()):
        write_meta_header_line(fp, k, str(meta[k]))


@dataclass(frozen=True)
class CsvMetaParseResult:
    meta: dict[str, str]
    first_non_meta_line: Optional[str]


def read_meta_header(fp: IO[str]) -> CsvMetaParseResult:
    """
    Read leading '# key=value' lines.

    Leaves the file positioned after the first non-meta line (which is returned).
    """
    meta: dict[str, str] = {}
    first_non: Optional[str] = None
    while True:
        pos = fp.tell()
        line = fp.readline()
        if line == "":
            first_non = None
            break
        s = line.strip("\r\n")
        if not s.startswith(META_PREFIX):
            first_non = line
            break
        # Allow "#key=value" and "# key=value"
        s2 = s[1:].lstrip()
        if not s2:
            continue
        if "=" not in s2:
            continue
        k, v = s2.split("=", 1)
        k = k.strip()
        v = _unescape_meta_value(v.strip())
        if k:
            meta[k] = v
        # keep reading meta lines

    if first_non is not None:
        # We've already consumed it; rewind so callers can re-read as normal CSV if desired.
        fp.seek(pos)
        # And read it again to return consistent content.
        first_non = fp.readline()

    return CsvMetaParseResult(meta=meta, first_non_meta_line=first_non)
