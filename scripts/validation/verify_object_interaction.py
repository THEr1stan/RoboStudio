"""Selected-color grasp, manual relocation, actual regrasp, and strict replay."""

import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

output = Path("artifacts/object-interaction-v073")
output.mkdir(parents=True, exist_ok=True)
rows = []


def finish(sim):
    for _ in range(100):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            return
    raise AssertionError(sim.failures[-3:])


for robot in ("r6", "gantry"):
    for index in (1, 2):
        sim = WorldSession(default_scene(robot))
        sim.command({"action": "select_object", "index": index})
        sim.command({"action": "start"})
        finish(sim)
        initial_step = sim.steps
        point = sim.cell.data.xpos[sim.cell.object_bodies[index]].tolist()
        sim.command({"action": "grab", "kind": "object", "index": index, "point_m": point})
        sim.command({"action": "drag", "position_m": [point[0], point[1], 0.18]})
        sim.command({"action": "drag", "position_m": [0.38, -0.13, 0.18]})
        sim.advance(150)
        held = sim.cell.data.xpos[sim.cell.object_bodies[index]].tolist()
        sim.command({"action": "release"})
        finish(sim)
        report = save_report(output, sim.report())
        identical = canonical(report) == canonical(replay_world(report))
        row = {
            "robot": robot,
            "color": sim.config["objects"][index]["color"],
            "object": index,
            "first_completion_step": initial_step,
            "held_position_m": held,
            "placements": report["placements"],
            "metrics": report["metrics"],
            "failures": report["failures"],
            "replay": "IDENTICAL" if identical else "MISMATCH",
            "source_hash": report["source_hash"],
            "run_id": report["run_id"],
        }
        rows.append(row)
        (output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
        print(json.dumps(row), flush=True)
        assert identical and len(sim.placements) == 2
        assert all(p["success"] and p["object"] == index for p in sim.placements)
