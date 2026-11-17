"""Export an explicit source allowlist, never local environments, experiment data or binaries."""

import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DIRECTORIES = (".github", "desktop", "src", "tests", "scripts", "docs", "tasks", "examples")
ROOT_FILES = (
    "README.md",
    "LICENSE",
    "CONTRIBUTING.md",
    "AGENTS.md",
    "THIRD_PARTY_NOTICES.md",
    ".gitignore",
    ".gitattributes",
    "pyproject.toml",
    "package.json",
    "requirements-dev.lock",
    "requirements-physics.lock",
    "Start-RoboStudio.cmd",
)
SUFFIXES = {
    ".py",
    ".pyi",
    ".typed",
    ".json",
    ".xml",
    ".html",
    ".css",
    ".js",
    ".mjs",
    ".cjs",
    ".cs",
    ".svg",
    ".md",
    ".toml",
    ".yml",
    ".yaml",
    ".ps1",
    ".cmd",
    ".txt",
    ".ico",
    ".png",
    ".manifest",
}
EXCLUDED = {"__pycache__", "node_modules", ".git", "Data", "bin", "obj", ".codex", ".agents"}


def source_files() -> list[Path]:
    selected = [ROOT / name for name in ROOT_FILES]
    for name in DIRECTORIES:
        for path in (ROOT / name).rglob("*"):
            rel = path.relative_to(ROOT)
            if not path.is_file() or path.is_symlink() or path.suffix not in SUFFIXES:
                continue
            if any(part in EXCLUDED or part.endswith(".egg-info") for part in rel.parts):
                continue
            # Publish one current snapshot; keep local historical records on disk.
            if rel.parts[:2] == ("docs", "history") or rel.as_posix() == "tasks/HISTORY.md":
                continue
            if rel.parts[:2] == ("docs", "releases") and path.name != "DESKTOP_V080.md":
                continue
            selected.append(path)
    return sorted(selected)


def main() -> None:
    output = ROOT / "dist"
    output.mkdir(exist_ok=True)
    archive = output / "RoboStudio-0.8.0-source.zip"
    entries = []
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
        for path in source_files():
            data = path.read_bytes()
            name = path.relative_to(ROOT).as_posix()
            entry = zipfile.ZipInfo("RoboStudio/" + name, date_time=(2026, 11, 17, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            bundle.writestr(entry, data)
            entries.append(
                {"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            )
    manifest = {
        "archive": archive.name,
        "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "file_count": len(entries),
        "files": entries,
    }
    (output / "RoboStudio-0.8.0-source-manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in manifest.items() if k != "files"}))


if __name__ == "__main__":
    main()
