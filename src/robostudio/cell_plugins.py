"""Strict trusted manifests. Identifiers dispatch only to bundled implementations."""

import json
from functools import lru_cache
from importlib.resources import files

from .core import ErrorCode
from .core.errors import require
from .registry.manifests import PluginManifest, parse_plugin


@lru_cache(maxsize=1)
def manifests() -> tuple[PluginManifest, ...]:
    data = json.loads(files("robostudio").joinpath("data", "cell_algorithms.json").read_text())
    return tuple(parse_plugin(item) for item in data)


def require_algorithm(name: str, capabilities: frozenset[str], damping: float = 0.035) -> None:
    match = next((p for p in manifests() if p.plugin_id == f"cell_{name}_v1"), None)
    require(match is not None, ErrorCode.NOT_FOUND, "unknown cell algorithm")
    assert match is not None
    require(
        all(cap.value in capabilities for cap in match.requires_capabilities.values),
        ErrorCode.UNSUPPORTED_CAPABILITY,
        "required algorithm capabilities missing",
    )
    match.validate_parameters({"damping": damping} if name in ("dls", "adaptive_dls") else {})
