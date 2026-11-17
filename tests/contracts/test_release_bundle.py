"""The public snapshot contains the current product, licenses and runnable examples."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def selected_files():
    spec = importlib.util.spec_from_file_location(
        "source_bundle", ROOT / "scripts/build/source_bundle.py"
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {path.relative_to(ROOT).as_posix() for path in module.source_files()}


def test_release_contains_license_and_getting_started():
    assert {
        "LICENSE",
        "README.md",
        "THIRD_PARTY_NOTICES.md",
        "docs/QUICK_START.md",
        "docs/SCENES.md",
        "docs/TROUBLESHOOTING.md",
        "examples/scenes/part_loading.robostudio.json",
        "desktop/windows/Program.cs",
        "scripts/build/windows.ps1",
        "src/robostudio/data/r6_compact.xml",
        "src/robostudio/ui/studio/studio.html",
    } <= selected_files()


def test_release_excludes_history_and_local_data():
    paths = selected_files()
    forbidden = ("artifacts/", "dist/", "build/", "docs/history/", ".venv", "Data/")
    assert not any(path.startswith(forbidden) for path in paths)
    assert "tasks/HISTORY.md" not in paths
    assert {path for path in paths if path.startswith("docs/releases/")} == {
        "docs/releases/DESKTOP_V080.md"
    }


def test_release_has_no_symlinks_or_missing_files():
    for name in selected_files():
        path = ROOT / name
        assert path.is_file() and not path.is_symlink()
        assert path.resolve().is_relative_to(ROOT.resolve())
