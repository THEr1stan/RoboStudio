"""Fixed desktop tasks, intentional failures and exact reruns; no generated code."""

import argparse
import json
import random
from pathlib import Path

from robostudio.cell_ik import solve
from robostudio.workcell import Cell, default_config, handle_request, run_cell


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/workcell-final-validation"))
    args = parser.parse_args()
    cases = []
    for robot in ("r6", "gantry"):
        standard = default_config(robot)
        cases.extend(
            [
                ("standard", standard),
                ("cross", {**standard, "object_xy": [0.34, -0.21], "destination_xy": [0.49, 0.19]}),
                ("slippery", {**standard, "mass_kg": 0.28, "friction": 0.05}),
                ("pd", {**standard, "controller": "pd_bias"}),
                ("pinv", {**standard, "solver": "pinv"}),
                ("transpose", {**standard, "solver": "transpose"}),
            ]
        )
        rng = random.Random(20261114)
        for i in range(4):
            cases.append(
                (
                    f"layout{i}",
                    {
                        **standard,
                        "object_xy": [
                            round(rng.uniform(0.34, 0.50), 3),
                            round(rng.uniform(-0.22, -0.09), 3),
                        ],
                        "destination_xy": [
                            round(rng.uniform(0.34, 0.50), 3),
                            round(rng.uniform(0.10, 0.24), 3),
                        ],
                    },
                )
            )
    results = []
    for name, config in cases:
        run = run_cell(config, args.output)
        replay = handle_request("replay", {"run_id": run["run_id"]}, args.output)
        assert replay["status"] == "IDENTICAL"
        if name in ("standard", "cross"):
            assert run["status"] == "SUCCEEDED"
        if name == "slippery":
            assert run["status"] == "FAILED"
        result = {
            "case": name,
            "robot": config["robot"],
            "run_id": run["run_id"],
            "status": run["status"],
            "error": run["error"],
            "metrics": run["metrics"],
            "replay": replay["status"],
        }
        results.append(result)
        print(json.dumps(result), flush=True)
    comparisons = []
    for robot in ("r6", "gantry"):
        cell = Cell(default_config(robot))
        for method in ("dls", "pinv", "transpose"):
            result = solve(cell, [0.42, -0.14, 0.073], method)
            comparisons.append(
                {
                    "robot": robot,
                    **{
                        key: result[key]
                        for key in (
                            "algorithm",
                            "iterations",
                            "status",
                            "position_error_m",
                            "budget",
                        )
                    },
                }
            )
        standard = next(r for r in results if r["robot"] == robot and r["case"] == "standard")
        baseline = next(r for r in results if r["robot"] == robot and r["case"] == "pd")
        assert standard["metrics"]["steps"] == baseline["metrics"]["steps"]
    report = {
        "tasks": len(results),
        "succeeded": sum(r["status"] == "SUCCEEDED" for r in results),
        "strict_replays": len(results),
        "results": results,
        "solver_comparison": comparisons,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
