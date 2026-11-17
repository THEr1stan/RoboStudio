"""Frozen implementation bodies and public adapters survive the v0.8 layout migration."""

import ast
import hashlib
import importlib
import json
from importlib.resources import files
from pathlib import Path

import pytest

BASELINE = json.loads(Path("tests/fixtures/layout_v076.json").read_text("utf-8"))


def normalized_ast(node):
    """Stable across Python 3.11..3.15 empty-field dump formatting changes."""
    if isinstance(node, ast.AST):
        return [
            type(node).__name__,
            {
                key: normalized_ast(value)
                for key, value in ast.iter_fields(node)
                if value is not None and value != []
            },
        ]
    if isinstance(node, list):
        return [normalized_ast(value) for value in node]
    return [type(node).__name__, repr(node)]


@pytest.mark.parametrize("name", BASELINE["simulation"])
def test_legacy_module_is_the_same_implementation(name):
    old = importlib.import_module("robostudio.studio_" + name)
    new = importlib.import_module("robostudio.simulation." + name)
    assert old is new


@pytest.mark.parametrize("name,expected", BASELINE["simulation"].items())
def test_simulation_computation_body_is_unchanged(name, expected):
    module = importlib.import_module("robostudio.simulation." + name)
    tree = ast.parse(Path(module.__file__).read_text("utf-8"))

    # Package-relative imports change; every non-import statement must stay identical.
    class WithoutImports(ast.NodeTransformer):
        def visit_ImportFrom(self, node):
            return None

        def visit_Import(self, node):
            return None

    body = json.dumps(normalized_ast(WithoutImports().visit(tree)), sort_keys=True)
    assert hashlib.sha256(body.encode()).hexdigest() == expected


@pytest.mark.parametrize("name,expected", BASELINE["unchanged"].items())
def test_trusted_assets_and_algorithms_are_byte_identical(name, expected):
    data = files("robostudio").joinpath(name).read_bytes().replace(b"\r\n", b"\n")
    assert hashlib.sha256(data).hexdigest() == expected


@pytest.mark.parametrize("old,entry", BASELINE["ui"].items())
def test_relocated_ui_preserves_content(old, entry):
    data = files("robostudio").joinpath(entry["path"]).read_bytes().replace(b"\r\n", b"\n")
    if entry["path"] == "ui/studio/studio.html":
        # Only the release label changes; compare all remaining bytes to the frozen UI.
        assert b"RoboStudio <b>0.8.0</b>" in data
        data = data.replace(b"RoboStudio <b>0.8.0</b>", b"RoboStudio <b>0.7.6</b>")
    assert hashlib.sha256(data).hexdigest() == entry["sha256"], old
