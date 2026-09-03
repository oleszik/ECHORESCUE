# ADR 0005: Replay as a versioned interface

## Status

Accepted

## Context

Replay files are committed evidence, test fixtures, and dashboard inputs.
Telemetry has expanded across local knowledge, constrained networking, Relay,
failure recovery, Smoke, and Thermal experiments. Unversioned structural
changes would silently break old artifacts or misrepresent missing data.

## Decision

Every replay declares `schema_version`. New capabilities reuse extensible
fields when compatible and increase the version only for a real interface
change. The dashboard validates an explicit supported-version set and supplies
safe display defaults for fields absent from older schemas. Regression tests
load historical committed replays and exercise benchmark normalization.

Current schema constants and serialization live in
[`replay.py`](../../src/echorescue/replay.py); validation and serving live in
[`dashboard.py`](../../src/echorescue/dashboard.py).

## Consequences

- Committed replays remain stable evidence rather than disposable output.
- Producers must choose schema changes deliberately and add compatibility tests.
- Consumers must distinguish a missing legacy field from a measured zero.
- Full snapshots favor auditability today but may require versioned delta
  encoding if map or fleet size grows substantially.
