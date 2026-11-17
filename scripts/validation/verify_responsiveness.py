"""Matched physics evidence for transport/render changes; works with the frozen 0.7.4 wheel."""

import argparse
import json
import math
from pathlib import Path

from robostudio import __version__
from robostudio.serialization import canonical, digest
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
rows = []
for robot in ("r6", "gantry"):
    sim = WorldSession(default_scene(robot))
    sim.command({"action": "start"})
    while sim.phase != "transfer" and sim.steps < 12000:
        sim.advance(25)
    assert sim.phase == "transfer"
    c = sim.cell
    point = c.data.site_xpos[c.tcp].tolist()
    selectable = {
        int(c.model.geom_bodyid[g])
        for g in c.robot_geoms
        if c.model.body_dofnum[c.model.geom_bodyid[g]] > 0
    }
    body = min(selectable, key=lambda b: math.dist(c.data.xpos[b], point))
    sim.command(
        {
            "action": "grab",
            "kind": "body",
            "index": body,
            "point_m": point,
        }
    )
    for i in range(10):
        sim.command(
            {
                "action": "drag",
                "position_m": [point[0] + 0.003 * (i % 3), point[1], point[2] + 0.002],
            }
        )
        sim.advance(20)
    sim.command({"action": "release"})
    while sim.steps < 12000:
        sim.advance(min(500, 12000 - sim.steps))
    report = sim.report()
    assert sim.goal_state == "satisfied", sim.failures
    assert canonical(report) == canonical(replay_world(report))
    (args.output / (robot + "-run.json")).write_text(json.dumps(report), encoding="utf-8")
    physical = {
        k: report[k]
        for k in (
            "frames",
            "trace",
            "sensors",
            "metrics",
            "actions",
            "events",
            "placements",
            "failures",
            "model_hash",
        )
    }
    rows.append(
        {
            "robot": robot,
            "version": __version__,
            "source_hash": report["source_hash"],
            "physical_digest": digest(physical),
            "goal": sim.goal_state,
            "cycles": sim.cycles,
            "metrics": report["metrics"],
            "replay": "IDENTICAL",
        }
    )
(args.output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
print(json.dumps(rows, indent=2))
