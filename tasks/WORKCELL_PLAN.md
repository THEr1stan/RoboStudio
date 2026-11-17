# Active: functional desktop manipulation / v0.4

Owner request (2026-11-14): replace disconnected demonstrations with a concrete,
interactive physical grasping task. This supersedes the previous next-model slice.

Changes: new `workcell.py` scene adapter and runtime, `cell_ik.py` traced solvers,
new workcell HTML/CSS/JS, renderer picking/scene geometry, additive HTTP endpoints,
contract/integration/browser tests, task records and delivery notes. Original MJCF,
contracts, old APIs and advanced workbench remain available. No irreversible action,
publication, hardware control, third-party models or new dependencies.

Contracts before implementation: explicit scene/robot joint mappings (nq != nv for
free objects), strict bounded scene inputs, world-frame SI targets, three bounded
IK algorithms with iteration evidence, physical grasp success from contact/object
height/landing, deterministic seeded reset and repeat runs. Algorithms share the
same initial state, tolerances and iteration budget. UI drag is kinematic preview;
execution uses torques and contact dynamics, never object attachment/teleportation.

Paper workflow: a cited trusted DLS reference connects to this task. User-authored
paper excerpts produce reviewable specifications only; no arbitrary generated code
execution or claims of original paper reproduction. Approval remains in product UI.

Validation: regressions, numerical solver/adapter tests, real grasp success/failure,
deterministic rerun, HTTP validation and browser interactions/screenshots.
