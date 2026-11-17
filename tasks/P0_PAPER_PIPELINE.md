# P0 — Paper ingestion & reviewed AlgorithmSpec (future; not active)

**Prerequisites:** M0/M1/M2 accepted, one target paper chosen based on clear numeric IK equations and license.

1. Import one text-layer PDF to an isolated paper workspace; record title, version, SHA256 and metadata.
2. Extract selected pages/formulas/pseudocode with human-checkable page citations.
3. Define a validated `AlgorithmSpec` for numerical IK, with required inputs/outputs, explicit coordinate conventions, equations, parameter values, claims and missing-item statuses.
4. Implement `robostudio paper inspect <pdf>`, `robostudio paper spec <paper-id>` and `robostudio paper approve <spec-id>`.
5. Expose incomplete/ambiguous items without manufacturing values. Approval freezes a versioned spec hash.
6. Unit tests: correct page references, invalid spec, missing parameter status, revision and approval audit.

**Do not implement in P0:** auto-run generated code, LLM auto-fix loops, full paper replication, cloud GUI.
