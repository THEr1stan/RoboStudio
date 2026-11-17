"""Blocked drag -> lift over obstacle -> release -> physical grasp, with strict replay."""

import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

output = Path("artifacts/drag-feedback-v074")
output.mkdir(parents=True, exist_ok=True)
rows = []
for robot in ("r6", "gantry"):
    cfg = default_scene(robot)
    cfg["obstacles"] = [{"xy": [0.42, -0.05], "size": [0.05, 0.02, 0.08], "yaw": 0}]
    sim = WorldSession(cfg)
    c = sim.cell
    point = c.data.xpos[c.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": point})
    state = sim.command({"action": "drag", "position_m": [0.42, 0.04, point[2]]})
    blocked = state["world"]["blocking"]
    assert blocked and blocked["kind"] == "obstacle"
    stopped = c.data.xpos[c.object_bodies[0]].tolist()
    for target in ([stopped[0], stopped[1], 0.25], [0.54, 0.09, 0.25], [0.54, -0.13, 0.18]):
        state = sim.command({"action": "drag", "position_m": target})
        assert state["world"]["blocking"] is None
        sim.advance(100)
    sim.command({"action": "release"})
    sim.advance(500)
    sim.command({"action": "start"})
    for _ in range(100):
        sim.advance(500)
        if sim.goal_state in ("satisfied", "blocked"):
            break
    report = save_report(output, sim.report())
    identical = canonical(report) == canonical(replay_world(report))
    row = {
        "robot": robot,
        "blocking": blocked,
        "state": sim.goal_state,
        "placements": sim.placements,
        "failures": sim.failures,
        "metrics": sim.metrics(),
        "replay": "IDENTICAL" if identical else "MISMATCH",
        "source_hash": report["source_hash"],
        "run_id": report["run_id"],
    }
    rows.append(row)
    (output / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(row, ensure_ascii=False), flush=True)
    assert identical and sim.goal_state == "satisfied" and sim.placements[-1]["success"]
