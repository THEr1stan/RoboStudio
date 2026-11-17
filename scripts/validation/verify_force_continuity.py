"""Record live task continuity under force; rerun with --output for matched versions."""

import argparse
import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True)
args = parser.parse_args()
sim = WorldSession(default_scene())
sim.command({"action": "start"})
while sim.phase != "transfer":
    sim.advance(25)
c = sim.cell
point = c.data.site_xpos[c.tcp].tolist()
body = int(c.model.site_bodyid[c.tcp])
before = {"step": sim.steps, "phase": sim.phase, "current": sim.current, "close": sim.close,
          "cycles": sim.cycles, "completed": sim.completed.copy()}
sim.command({"action": "grab", "kind": "body", "index": body, "point_m": point})
sim.command({"action": "drag", "position_m": [point[0]+.02, point[1], point[2]+.005]})
sim.advance(80)
force = sim.force_n
sim.command({"action": "release"})
released = {"step": sim.steps, "phase": sim.phase, "current": sim.current, "close": sim.close,
            "cycles": sim.cycles, "completed": sim.completed.copy(), "force_before_release_n": force}
while sim.steps < 16000:
    sim.advance(min(500, 16000-sim.steps))
report = sim.report()
assert canonical(report) == canonical(replay_world(report))
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
(out / "run.json").write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")
summary = {"before_force": before, "released": released, "final_goal": sim.goal_state,
           "final_cycles": sim.cycles, "placements": sim.placements,
           "failures": sim.failures, "replay": "IDENTICAL", "source_hash": report["source_hash"]}
(out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(summary, ensure_ascii=False, indent=2))
