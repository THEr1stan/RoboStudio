"""Verify a saved continuous-world artifact without executing external code."""
import argparse
import json
from pathlib import Path

from robostudio.serialization import canonical, digest
from robostudio.studio_world import replay_world

parser = argparse.ArgumentParser()
parser.add_argument("--record", type=Path, required=True)
args = parser.parse_args()
old = json.loads(args.record.read_text(encoding="utf-8"))
assert old["schema"] == "studio-world-v1", "expected world record"
assert old["run_id"] == "studio_" + digest({k:v for k,v in old.items() if k != "run_id"})[:24]
new = replay_world(old)
assert canonical(old) == canonical(new), "source/environment/data differs"
print(json.dumps({"status":"IDENTICAL", "run_id":old["run_id"], "steps":old["metrics"]["steps"]}))
