"""Untrusted paper excerpts become reviewable data, never executable instructions."""

import base64
import io
import json
import zipfile
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from .cell_ik import SOLVERS
from .core import ErrorCode
from .core.errors import require
from .registry.manifests import record
from .serialization import digest


def draft_spec(payload: Any) -> dict[str, Any]:
    data = record(payload, "paper", {"title", "source_url", "page", "excerpt", "method"})
    for key, limit in (("title", 200), ("source_url", 1000), ("excerpt", 3000)):
        value = data[key]
        require(
            isinstance(value, str) and 0 < len(value.strip()) <= limit,
            ErrorCode.INVALID_VALUE,
            f"invalid {key}",
        )
    url = urlsplit(str(data["source_url"]))
    require(
        url.scheme in ("https", "http") and bool(url.hostname),
        ErrorCode.INVALID_VALUE,
        "expected source URL",
    )
    require(
        type(data["page"]) is int and 1 <= data["page"] <= 9999,
        ErrorCode.INVALID_VALUE,
        "invalid page",
    )
    require(
        isinstance(data["method"], str) and data["method"] in SOLVERS,
        ErrorCode.INVALID_VALUE,
        "unsupported method",
    )
    spec = {
        "schema": "AlgorithmSpec-v1",
        "status": "NEEDS_HUMAN_REVIEW",
        "title": data["title"],
        "method": data["method"],
        "evidence": [
            {
                "source_url": data["source_url"],
                "page": data["page"],
                "excerpt": data["excerpt"],
                "verified": False,
            }
        ],
        "assumptions": [
            "World-frame SI position and spatial rotation error",
            "Joint bounds, step clipping and line search are project choices",
            "No claim of reproducing the paper's original experimental protocol",
        ],
        "missing": [
            "Human verification of equations and source pages",
            "Algorithm parameters",
            "Approved generation and isolated candidate execution",
        ],
        "code_generated": False,
        "executable": False,
        "evaluation_level": "NOT_EVALUATED",
    }
    return {**spec, "spec_hash": digest(spec)}


def generate_candidate(payload: Any, output: Path) -> dict[str, Any]:
    """Export a bounded method template only after explicit review. Never execute it."""
    data = record(payload, "candidate", {"source", "approved"})
    require(data["approved"] is True, ErrorCode.INVALID_VALUE, "human spec approval required")
    spec = draft_spec(data["source"])
    spec["status"] = "USER_APPROVED_FOR_TEMPLATE_GENERATION"
    spec["code_generated"] = True
    spec.pop("spec_hash")
    spec["spec_hash"] = digest(spec)
    method = str(spec["method"])
    expressions = {
        "dls": "return j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(j.shape[0]), e)",
        "pinv": "return np.linalg.pinv(j, rcond=1e-5) @ e",
        "transpose": (
            "a = j @ j.T @ e\n    return (float(e @ a) / max(1e-16, float(a @ a))) * j.T @ e"
        ),
    }
    code = (
        '"""UNEXECUTED template candidate; requires isolated validation before integration."""\n'
        "import numpy as np\n\n\ndef step(jacobian, error, damping=0.035):\n"
        "    j = np.asarray(jacobian, dtype=float)\n"
        "    e = np.asarray(error, dtype=float)\n"
        "    if j.ndim != 2 or e.shape != (j.shape[0],):\n"
        '        raise ValueError("Jacobian/error dimensions differ")\n'
        "    if not np.isfinite(j).all() or not np.isfinite(e).all():\n"
        '        raise ValueError("Non-finite input")\n'
        "    if not 0.001 <= damping <= 0.5:\n"
        '        raise ValueError("Damping out of bounds")\n'
        f"    {expressions[method]}\n"
    )
    contents = {
        "candidate.py": code,
        "AlgorithmSpec.json": json.dumps(spec, ensure_ascii=False, indent=2),
        "VALIDATION_REQUIRED.md": (
            "# Generated, not executed\n\nThis is a fixed supported-method template, "
            "not LLM paper extraction.\n"
            "The excerpt is retained as unverified evidence. "
            "Source statements do not become code.\n"
            "Required before integration: isolated runner with no network/credentials, "
            "bounded CPU, memory,\n"
            "time and filesystem; numerical and failure tests; matched baseline task evaluation.\n"
            "No host execution or plugin registration has occurred. "
            "The .035 damping is a project assumption.\n"
        ),
    }
    identifier = "candidate_" + digest(contents)[:24]
    directory = (output / "candidates" / identifier).resolve()
    require(directory.is_relative_to(output.resolve()), ErrorCode.INVALID_VALUE, "candidate path")
    directory.mkdir(parents=True, exist_ok=True)
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, content in contents.items():
            (directory / name).write_text(content, encoding="utf-8")
            archive.writestr(name, content)
    return {
        "candidate_id": identifier,
        "status": "GENERATED_NOT_EXECUTED",
        "archive_base64": base64.b64encode(buffer.getvalue()).decode("ascii"),
        "files": list(contents),
        "spec": spec,
    }
