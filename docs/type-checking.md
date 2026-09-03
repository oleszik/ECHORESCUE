# Type-checking policy

EchoRescue uses mypy 1.18.2 as a pinned development and CI dependency. Run the
configured check from the repository root:

```bash
python -m mypy src
```

The current policy requires annotations for every checked function, checks
function bodies, rejects implicit Optional values, and enables unreachable,
redundant-cast, unused-ignore, and strict-equality diagnostics. It covers 26
production modules, including the simulation entry points, data models,
planning, coordination, mapping, sensors, communication, network transport,
telemetry, replay, dashboard, and CLI.

## Controlled incremental exception

Files named `benchmark.py` or `*_benchmark.py` are temporarily excluded. Their
public functions are annotated, but their deeply heterogeneous JSON assembly
uses `dict[str, object]` extensively. Bringing those adapters under the same
policy requires introducing explicit result schemas or TypedDict definitions;
doing that safely is separate from the simulation-core slices.

The v0.6 `scaling_benchmark.py` adapter follows this existing boundary. Its
configuration, runtime, coordination, replay, dashboard server, and CLI
dependencies remain inside the checked production core; only heterogeneous
benchmark aggregation is excluded.

The exception is narrow and visible in `pyproject.toml`. It does not use global
`ignore_errors`, per-line suppressions, or missing-import suppression. New
simulation behavior should be implemented in checked core modules rather than
inside an excluded benchmark adapter.

The next typing step is to define shared JSON value and benchmark-profile
schemas, migrate one benchmark family at a time, and then remove the exclusion.
