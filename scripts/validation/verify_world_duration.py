"""Cross the old 130 second stop limit and verify bounded history and exact replay."""
import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world

sim = WorldSession(default_scene("gantry"))
sim.command({"action": "start"})
while sim.steps < 90000:
    sim.advance(500)
assert sim.goal_state == "satisfied" and sim.cycles == 1
assert len(sim.frames) <= 1000 and len(sim.sensor_records) <= 40
report = save_report(Path("artifacts/world-final-validation"), sim.report())
assert canonical(report) == canonical(replay_world(report))
summary = {"steps":sim.steps, "time_s":sim.cell.data.time, "goal_state":sim.goal_state,
           "cycles":sim.cycles, "retained_frames":len(sim.frames),
           "retained_sensor_images":len(sim.sensor_records), "identical":True,
           "run_id":report["run_id"], "source_hash":report["source_hash"]}
Path("artifacts/world-final-validation/duration.json").write_text(json.dumps(summary,indent=2))
print(json.dumps(summary),flush=True)
