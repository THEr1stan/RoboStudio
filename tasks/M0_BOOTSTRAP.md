# M0 — Engineering foundation (Codex task card)

**Goal:** A cleanly installable Python package with typed robotics contracts, capability-driven plugin/robot manifests, deterministic fake simulation, CLI and CI.

## Required implementation
1. Inspect existing repo and prepare a small work plan. Use `src/` layout and a Python 3.11+ baseline.
2. Implement contracts: RobotSpec, RobotState, RobotReference, RobotCommand, Pose, CapabilitySet, RunResult/RunStatus, SimulationBackend protocol. Include explicit units/frame IDs.
3. Create manifest schema with `api_version`, `robot_id`, `family`, `joint_groups`, `actuator_mapping`, `capabilities` and model reference. Add one valid and one invalid fixture.
4. Implement registry and CLI commands `robostudio list robots`, `robostudio list plugins`, `robostudio validate robot <manifest>`, and `robostudio demo fake --seed 42`.
5. Fake backend supports load/reset/observe/apply/step/close; run twice with same seed produces equal observations and run metadata.
6. Emit run metadata to gitignored `artifacts/`: run_id, scenario_id, config_hash, seed, status, metrics; no fabricated time/performance results.
7. Create `docs/ARCHITECTURE.md`, `docs/adr/0001-core-backend-boundary.md`, README, lint/type check/pytest CI.

## Contract tests
- Invalid manifest: missing ID, invalid joint mapping, duplicate IDs, unrecognized version/capability, meaningful errors.
- Core imports no MuJoCo/ROS/LLM/GUI SDKs.
- Registry supports adding a dummy plugin without Core edits.
- Fake backend lifecycle and deterministic fixed-seed output.
- Invalid state dimension or frame mismatch returns explicit error.

## Non-goals
No MuJoCo, mechanical arm IK, paper import, LLM, generated plugin execution, ROS 2, navigation, GUI.

## Done means
`pip install -e .` (or agreed uv equivalent) succeeds from fresh environment, documented CLI runs, tests pass, CI passes; report exact commands and known limitations. Do not start M1.
