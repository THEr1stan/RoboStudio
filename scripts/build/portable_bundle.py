"""Package the verified portable app without its local profile, exports, logs or experiments."""

import hashlib
import json
import zipfile
from pathlib import Path

root = Path(__file__).resolve().parents[2]
app = root / "dist/RoboStudio-0.8.0-win-x64"
output = app.parent / (app.name + ".zip")
entries = []
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
    for path in sorted(app.rglob("*")):
        rel = path.relative_to(app)
        if not path.is_file() or any(p in {"Data", "__pycache__"} for p in rel.parts):
            continue
        if path.suffix in {".pyc", ".log", ".pid"}:
            continue
        if rel.parts[:2] == ("docs", "history"):
            continue
        if rel.parts[:2] == ("docs", "releases") and path.name != "DESKTOP_V080.md":
            continue
        data = path.read_bytes()
        archive.write(path, app.name + "/" + rel.as_posix())
        entries.append({"path": rel.as_posix(), "sha256": hashlib.sha256(data).hexdigest()})
with zipfile.ZipFile(output) as archive:
    assert archive.testzip() is None
manifest = {
    "archive": output.name,
    "bytes": output.stat().st_size,
    "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    "file_count": len(entries),
    "files": entries,
}
(root / "dist/RoboStudio-0.8.0-win-x64-manifest.json").write_text(
    json.dumps(manifest, indent=2), encoding="utf-8"
)
print(json.dumps({key: value for key, value in manifest.items() if key != "files"}))
