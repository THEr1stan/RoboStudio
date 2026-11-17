"""Bundle the active CPython runtime and the two existing physics dependencies, offline."""

import argparse
import importlib.metadata
import json
import shutil
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--destination", type=Path, required=True)
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
destination = args.destination.resolve()
if not destination.is_relative_to(repo / "dist"):
    parser.error("destination must be inside this repository's dist directory")
if sys.platform != "win32" or sys.version_info[:2] != (3, 12):
    parser.error("portable desktop packaging currently requires Windows x64 / CPython 3.12")
for name, expected in (("mujoco-core", "4.0.1"), ("numpy", "2.5.4")):
    if importlib.metadata.version(name) != expected:
        parser.error(f"expected existing dependency {name}=={expected}")
runtime = destination / "runtime"
if runtime.exists():
    parser.error("runtime already exists; use a new output folder to avoid stale packaged files")
runtime.mkdir(parents=True)
base = Path(sys.base_prefix)
for path in base.iterdir():
    if path.suffix.lower() in (".dll", ".exe") or path.name == "LICENSE.txt":
        shutil.copy2(path, runtime / path.name)
ignore = shutil.ignore_patterns(
    "__pycache__",
    "*.pyc",
    "site-packages",
    "test",
    "tests",
    "idlelib",
    "ensurepip",
    "tkinter",
    "turtledemo",
)
shutil.copytree(base / "Lib", runtime / "Lib", ignore=ignore)
shutil.copytree(
    base / "DLLs", runtime / "DLLs", ignore=shutil.ignore_patterns("_tkinter.pyd", "tcl*", "tk*")
)
site = runtime / "Lib/site-packages"
site.mkdir()
copied = set()
for name in ("mujoco-core", "numpy"):
    distribution = importlib.metadata.distribution(name)
    for record in distribution.files or []:
        rel = Path(str(record))
        if ".." in rel.parts or rel.parts[0] in copied:
            continue
        source = Path(distribution.locate_file(rel.parts[0]))
        target = site / rel.parts[0]
        if source.is_dir():
            shutil.copytree(
                source,
                target,
                ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "tests", "test"),
            )
        elif source.is_file():
            shutil.copy2(source, target)
        copied.add(rel.parts[0])
shutil.copytree(
    repo / "src/robostudio",
    site / "robostudio",
    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
)
# Some experiment reports query importlib.metadata rather than __version__.
metadata = site / "robostudio-0.8.0.dist-info"
metadata.mkdir()
(metadata / "METADATA").write_text(
    "Metadata-Version: 2.1\nName: robostudio\nVersion: 0.8.0\n", encoding="utf-8"
)
(runtime / "python312._pth").write_text(
    ".\nDLLs\nLib\nLib/site-packages\nimport site\n", encoding="utf-8"
)
manifest = {
    "python": sys.version,
    "platform": sys.platform,
    "application": "0.8.0",
    "dependencies": {name: importlib.metadata.version(name) for name in ("mujoco-core", "numpy")},
}
(destination / "runtime-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
print(json.dumps(manifest))
