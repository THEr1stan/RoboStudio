"""Matched baseline/live tasks, real disturbance recovery and strict input replay."""
import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_runtime import run_experiment
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

output = Path("artifacts/world-final-validation")
output.mkdir(parents=True, exist_ok=True)
rows = []
for robot in ("r6", "gantry"):
    for task in ("single", "sort", "stack"):
        cfg = default_scene(robot)
        cfg["task"] = task
        baseline = run_experiment(cfg)
        sim = WorldSession(cfg)
        sim.command({"action":"start"})
        for _ in range(200):
            sim.advance(500)
            if sim.goal_state in ("satisfied", "blocked"):
                break
        initial = {"state":sim.goal_state, "metrics":sim.metrics()}
        if sim.goal_state == "satisfied":
            index = 0 if task == "single" else 2
            p = sim.cell.data.xpos[sim.cell.object_bodies[index]].tolist()
            sim.command({"action":"grab", "kind":"object", "index":index, "point_m":p})
            for target in ([p[0],p[1],.20], [.48,-.18,.20], [.48,-.18,.05]):
                sim.command({"action":"drag", "position_m":target})
                for _ in range(3):
                    sim.advance(250)
            displaced = sim.cell.data.xpos[sim.cell.object_bodies[index]].tolist()
            sim.command({"action":"release"})
            for _ in range(200):
                sim.advance(500)
                if sim.goal_state in ("satisfied", "blocked"):
                    break
        else:
            displaced = None
        report = save_report(output, sim.report())
        replay = replay_world(report)
        row = {"robot":robot, "task":task, "baseline_status":baseline["status"],
               "baseline_metrics":baseline["metrics"], "initial":initial,
               "displaced_position_m":displaced, "recovery_state":sim.goal_state,
               "world_metrics":report["metrics"], "cycles":sim.cycles,
               "identical":canonical(report)==canonical(replay), "run_id":report["run_id"]}
        rows.append(row)
        print(json.dumps(row), flush=True)
        (output/"report.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
assert all(r["identical"] for r in rows)
