"""Export portable industrial layouts and measured, strictly replayable task records."""

import argparse
import json
import zipfile
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_documents import scene_document
from robostudio.studio_presets import industrial_scenes
from robostudio.studio_world import WorldSession, replay_world

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=Path("artifacts/industrial-scenes-v076"))
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
rows = []
files = []
for preset in industrial_scenes():
    doc = scene_document(preset["document"])
    path = args.output / (preset["id"] + ".robostudio.json")
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
    files.append(path)
    sim = WorldSession(doc["config"])
    sim.command({"action": "start"})
    for _ in range(100):
        sim.advance(500)
        assert not sim.failures, sim.failures
        if sim.goal_state == "satisfied":
            break
    assert sim.goal_state == "satisfied"
    report = save_report(args.output, sim.report())
    assert report["metrics"]["placed"] == (1 if doc["config"]["task"] == "single" else 3)
    assert report["metrics"]["bilateral_steps"] > 100
    assert report["metrics"]["collision_steps"] == 0
    assert all(p["success"] and p["camera_verified"] for p in report["placements"])
    assert canonical(report) == canonical(replay_world(report))
    record = args.output / report["run_id"] / "run.json"
    row = {
        "id": preset["id"],
        "name": doc["name"],
        "scene_file": path.as_posix(),
        "record": record.as_posix(),
        "replay_command": (
            ".venv/Scripts/python.exe scripts/replay_world.py --record " + record.as_posix()
        ),
        "replay": "IDENTICAL",
        "robot": doc["config"]["robot"],
        "metrics": report["metrics"],
        "placements": report["placements"],
        "final_objects": report["frames"][-1]["objects"],
        "sampled_plans": sum(p["iterations"] > 0 for p in report["plans"]),
        "seed": report["seed"],
        "application_version": report["application_version"],
        "source_hash": report["source_hash"],
        "model_hash": report["model_hash"],
        "config_hash": report["config_hash"],
        "task_hash": report["task_hash"],
    }
    rows.append(row)
    print(json.dumps(row, ensure_ascii=False), flush=True)
(args.output / "summary.json").write_text(
    json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
)
with zipfile.ZipFile(
    args.output / "industrial-scenes-v076.zip", "w", zipfile.ZIP_DEFLATED
) as bundle:
    for path in files:
        bundle.write(path, path.name)
