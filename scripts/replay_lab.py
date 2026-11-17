"""Verify an exported lab artifact, then recompute its full contents exactly."""

import argparse
import json
from pathlib import Path

from robostudio.serialization import canonical, digest
from robostudio.studio_lab import run_lab


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record", type=Path, required=True)
    args = parser.parse_args()
    if args.record.stat().st_size > 100_000_000:
        raise ValueError("record too large")
    old = json.loads(args.record.read_text(encoding="utf-8"))
    body = {key: value for key, value in old.items() if key != "lab_id"}
    if "lab_" + digest(body)[:24] != old["lab_id"]:
        raise ValueError("artifact integrity mismatch")
    new = run_lab(**old["replay_request"])
    if canonical(new) != canonical(old):
        raise ValueError("replay differs: source, environment or computation changed")
    print(json.dumps({"lab_id": old["lab_id"], "status": "IDENTICAL"}))


if __name__ == "__main__":
    main()
