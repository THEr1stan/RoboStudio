"""Canonical JSON and local, nonexecuting file access."""

import hashlib
import json
from pathlib import Path

from .core import ErrorCode, RoboStudioError


def canonical(value: object) -> str:
    return json.dumps(
        value, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":")
    )


def digest(value: object) -> str:
    return hashlib.sha256(canonical(value).encode("utf-8")).hexdigest()


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise RoboStudioError(ErrorCode.INVALID_MANIFEST, "duplicate JSON key", key)
        result[key] = value
    return result


def _nonfinite(value: str) -> object:
    raise RoboStudioError(ErrorCode.INVALID_MANIFEST, f"invalid JSON number: {value}")


def parse_json(text: str) -> object:
    try:
        return json.loads(text, object_pairs_hook=_unique_object, parse_constant=_nonfinite)
    except (ValueError, RecursionError) as error:
        if isinstance(error, RoboStudioError):
            raise
        raise RoboStudioError(ErrorCode.INVALID_MANIFEST, str(error), "json") from error


def read_json(path: Path) -> object:
    try:
        return parse_json(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeError) as error:
        raise RoboStudioError(ErrorCode.IO_ERROR, str(error), str(path)) from error
