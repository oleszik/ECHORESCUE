# Portfolio dashboard

The built-in dashboard is a read-only viewer for recorded simulation telemetry.
Running it without arguments opens the four-drone seed-44 mission immediately:

```bash
python -m echorescue.dashboard
```

Open <http://127.0.0.1:8000>.  `--host`, `--port`, `--replay` and `--benchmark`
remain supported.  Existing JSON replays can also be opened with the dashboard's
Upload replay control; the file stays in the browser and is not uploaded to the
server.

The public deployment is available at
[https://oleszik.github.io/ECHORESCUE/](https://oleszik.github.io/ECHORESCUE/).
It is built from the same five replay artifacts by the dedicated GitHub Pages
workflow. The existing
[ChatGPT Site deployment](https://echorescue-mission-control.oleszik.chatgpt.site)
remains available as a fallback.

![Desktop mission-control view](assets/v0.13-portfolio-desktop.png)

The narrow layout is captured in
[`assets/v0.13-portfolio-mobile.png`](assets/v0.13-portfolio-mobile.png).

## Recorded demo catalog

| Scenario | Artifact | Scope |
| --- | --- | --- |
| Multi-Agent Exploration (default) | `replays/seed_44_4_agents.json` | 4 drones, 1 floor, seed 44 |
| Multi-Floor Mission | `replays/seed_68_multi_floor.json` | 4 drones, 3 floors, seed 68 |
| Network & Relay | `replays/seed_7_network_aware.json` | 2 drones, constrained network-aware Relay, seed 7 |
| Uncertain Perception | `replays/seed_11000_uncertain.json` | 2 drones, medium noise, uncertainty-aware planning, development seed 11000 |
| ROS 2 Closed Loop | `replays/v0.13-ros2-closed-loop.json` | 1 drone, 1 floor, recorded simulator-integration reference |

The Network & Relay tile intentionally uses the smaller supported two-drone
recording instead of the 6.8 MB four-drone predictive artifact.  The ROS 2 tile
is explicitly a one-drone process-integration proof, not a fleet or hardware
flight demonstration.

The two generated catalog artifacts can be reproduced without changing any
simulation algorithm or frozen holdout data:

```bash
python -m echorescue --seed 11000 --drones 2 --width 13 --height 9 \
  --survivors 3 --uncertainty-profile medium_noise \
  --planning-variant uncertainty-aware \
  --replay-out replays/seed_11000_uncertain.json

python -m echorescue.closed_loop_demo \
  --report-out .local-demo/v0.13-catalog-report.json \
  --replay-out replays/v0.13-ros2-closed-loop.json
```

## Evidence shown on the project overview

The uncertainty chart is read from the committed
`benchmarks/uncertainty_holdout.json`, not hand-entered.  It contains 160 runs:
four profiles, two planners and 20 holdout seeds per group.  Both planners have
mission success rates of 100%, 70%, 25% and 0% for clean, low, medium and high
noise.  Therefore the dashboard states that uncertainty-aware planning did not
improve success rate.  The clean paired successful-run duration difference is
-10.8 steps (aware minus naive, n=20, 95% CI [-26.55, -2.20]).  Interpretation
and all other intervals remain in `docs/v0.12-uncertain-perception.md`.

ROS 2 status comes from the process-level matrix documented in
`docs/v0.13-ros2-bridge.md`: normal completion, interrupted observations,
interrupted state, mission-process loss, duplicate/delayed messages and session
restart.  It is labeled simulator integration and never presented as real
flight validation.

## Compatibility and presentation rules

Scenario changes pause playback and reset the frame, timeline, floor and map
view.  Demo failures appear inline with a retry action.  Missing optional fields
are labeled `Not recorded`; the UI does not invent zero collisions, a full
battery or a successful connection.  Operator, base and per-agent knowledge
views remain distinct, and Ground Truth is exposed only by an explicitly labeled
debug view when the replay contains one.

Styles and scripts use relative asset/API URLs so the page assets remain valid
below a hosting path.  There is no `.openai/hosting.json` and v0.13 does not add
deployment configuration or a frontend framework dependency.

## Verification

The final dashboard was rendered with installed Google Chrome 152 at 1600×1100
and 390×844. Browser-driven checks exercised Play/Pause, pause stability,
timeline scrubbing, 4× playback, restart, Multi-Floor selection, a floor change,
the one-drone ROS 2 scenario and in-browser JSON upload. All five catalog
scenarios rendered their expected 4/4/2/2/1 agent-card counts without the fatal
error state, and no captured interaction produced an uncaught browser error.
The dedicated Computer-Use surface exposed no browser, so the same installed
Chrome binary was driven through its local DevTools protocol and inspected from
the generated screenshots. The repository verification completed with 324 tests
passed, 1 skipped and 42 subtests passed; `python -m mypy src`, JavaScript syntax
checking and Python bytecode compilation also completed successfully.
