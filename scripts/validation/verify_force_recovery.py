"""Replay the recorded pre-fix failure with the same inputs and step budget."""

import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_world import replay_world

before = json.loads(Path("artifacts/force-repeat-before.json").read_text())
after = replay_world(before)
assert after["world"]["goal_state"] == "satisfied", after["failures"][-3:]
assert canonical(after) == canonical(replay_world(after))
path = Path("artifacts") / (after["run_id"] + ".json")
path.write_text(json.dumps(after, ensure_ascii=False, indent=2), encoding="utf-8")
summary = {
    "seed": before["config"]["seed"],
    "matched_steps": after["metrics"]["steps"],
    "same_inputs": before["actions"] == after["actions"],
    "before": {"goal": before["world"]["goal_state"], "failures": len(before["failures"]),
               "source_hash": before["source_hash"]},
    "after": {"goal": after["world"]["goal_state"], "failures": len(after["failures"]),
              "metrics": after["metrics"], "recoveries": after["recoveries"],
              "source_hash": after["source_hash"], "record": str(path), "replay": "IDENTICAL"},
}
Path("artifacts/force-recovery-comparison.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
