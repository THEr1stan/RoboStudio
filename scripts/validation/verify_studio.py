"""Fixed, disclosed test matrix. Every row is real physics, including failed tasks."""

import argparse
import copy
import json
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_runtime import run_experiment
from robostudio.studio_scene import default_scene


def cases():
    result = []
    for robot in ("r6", "gantry"):
        base = default_scene(robot)
        for task in ("single", "sort", "stack"):
            result.append((f"{robot}_{task}", {**copy.deepcopy(base), "task": task}))
        for planner in ("direct", "rrt_connect"):
            cfg = copy.deepcopy(base)
            cfg.update(
                planner=planner,
                obstacles=[{"xy": [0.43, 0.015], "size": [0.035, 0.045, 0.115], "yaw": 0}],
            )
            result.append((f"{robot}_obstacle_{planner}", cfg))
        cfg = copy.deepcopy(base)
        cfg["objects"][0].update(mass=0.28, friction=0.05)
        result.append((f"{robot}_slippery_heavy", cfg))
        for key, value in (
            ("controller", "pd_bias"),
            ("solver", "pinv"),
            ("solver", "transpose"),
            ("observation", "truth"),
        ):
            result.append((f"{robot}_{value}", {**copy.deepcopy(base), key: value}))
        cfg = copy.deepcopy(base)
        cfg["objects"][0].update(shape="cylinder", size=[0.022, 0.022, 0.024])
        result.append((f"{robot}_cylinder", cfg))
        cfg = copy.deepcopy(base)
        cfg["objects"][0].update(yaw=0.5, size=[0.028, 0.016, 0.02])
        result.append((f"{robot}_rotated_rectangle", cfg))
        cfg = copy.deepcopy(base)
        cfg.update(noise_mm=3, dropout=0.2, latency_ms=300)
        result.append((f"{robot}_noisy_delayed_camera", cfg))
        for seed in (1, 2):
            cfg = copy.deepcopy(base)
            cfg["seed"] = seed
            cfg["objects"][0]["xy"] = [0.39 + seed * 0.025, -0.20 + seed * 0.035]
            result.append((f"{robot}_layout_seed_{seed}", cfg))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/studio-validation"))
    parser.add_argument("--replay", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, config in cases():
        result = run_experiment(config)
        replayed = None
        if args.replay:
            repeated = run_experiment(config, result["actions"], result["metrics"]["steps"])
            replayed = canonical(result) == canonical(repeated)
            if not replayed:
                raise RuntimeError(f"replay mismatch: {name}")
        save_report(args.output, result)
        row = {
            "case": name,
            "run_id": result["run_id"],
            "status": result["status"],
            "error": result["error"],
            "metrics": result["metrics"],
            "source_hash": result["source_hash"],
            "replay_identical": replayed,
        }
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        report = {
            "cases": len(rows),
            "successes": sum(r["status"] == "SUCCEEDED" for r in rows),
            "replayed": sum(r["replay_identical"] is True for r in rows),
            "results": rows,
        }
        (args.output / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), "utf-8"
        )
    print(json.dumps({key: value for key, value in report.items() if key != "results"}), flush=True)


if __name__ == "__main__":
    main()
