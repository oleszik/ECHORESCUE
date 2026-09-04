# Architecture decision records

EchoRescue records durable architecture choices as short ADRs. An accepted ADR
describes the current implementation; later changes should supersede an ADR
rather than silently rewriting its decision.

| ADR | Status | Decision |
| --- | --- | --- |
| [0001](0001-deterministic-simulation.md) | Accepted | Seeded and hash-derived deterministic simulation |
| [0002](0002-deterministic-coordination-resolution.md) | Accepted | Stable conflict and assignment resolution |
| [0003](0003-ground-truth-and-agent-knowledge.md) | Accepted | Separation of Ground Truth from agent/operator knowledge |
| [0004](0004-headless-core-and-replay-dashboard.md) | Accepted | Headless simulation with read-only visualization |
| [0005](0005-versioned-replay-interface.md) | Accepted | Replay JSON as a versioned compatibility boundary |
| [0006](0006-generic-n-agent-coordination.md) | Accepted | One deterministic collection-oriented core for 1–8 agents |
| [0007](0007-evidence-based-survivor-hypotheses.md) | Accepted | Confidence-weighted Survivor hypotheses without Ground-Truth leakage |
| [0008](0008-mutable-environment-observation-driven-replanning.md) | Accepted | Mutable Ground Truth with observation-driven path invalidation and replanning |
| [0009](0009-role-task-ownership-recovery.md) | Accepted | Deterministic N-agent role and single-owner task recovery semantics |
