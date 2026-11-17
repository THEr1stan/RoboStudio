# P1 — Reviewed IK plugin generation & isolation (future; not active)

**Prerequisites:** Approved AlgorithmSpec and stable IK contract.

- Use Codex in a supervised mode first, or an opt-in configurable provider, with fixed prompt templates and explicit output path constraints.
- Generate implementation plan first, then candidate IK plugin + tests in disposable workspace; preserve source provenance and all diffs.
- Verify the candidate in a resource-constrained nonprivileged isolated executor with no network, no secrets, no host Docker socket.
- Block execution when isolation is not available. Never import candidate code into the main app before successful test and review.
- Report contract/math/timeout/resource failures. Successful candidates go into Arena as `VERIFIED_L0`; optional transfer tests then give L1 status.
- Human review and explicit confirmation required before promoting a candidate to trusted reusable plugin.
