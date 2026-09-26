# EchoRescue Gazebo portfolio demo

The portfolio demo runs the previously accepted v0.15.0 known-map flight with
its real single ArduPilot Gazebo Iris, ROS 2 waypoint mission, and evaluator.
It uses the existing two-room indoor world, doorway, and obstacle; the
deterministic planner generates a route through the doorway and back. The
dedicated Gazebo GUI preset frames the full indoor layout from an elevated
oblique camera in a 1920 × 1080 window.

This is one simulated drone, not a multi-drone Gazebo demonstration. It does
not include survivor markers or survivor detection. The separately accepted
v0.16.1 rendered-camera search is a later development milestone; this v0.15.0
flight and its screenshot are not evidence of that perception capability.

After completing the pinned ROS 2 Jazzy, Gazebo Harmonic, and ArduPilot SITL
setup described in the [3D integration baseline](v0.14-3d-integration-baseline.md),
source the simulation environment and run these commands from the repository
root:

```bash
source /opt/ros/jazzy/setup.bash
source .venv-sim/bin/activate
export GZ_VERSION=harmonic
export ARDUPILOT_HOME="$HOME/sim-src/ardupilot"
export ARDUPILOT_GAZEBO_HOME="$HOME/sim-src/ardupilot_gazebo"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
source ros2_ws/install/setup.bash
./scripts/run_portfolio_demo.sh
```

The real mission and Gazebo GUI run together. Capture the screenshot while the
mission is in progress; the integration runner closes its owned processes when
the bounded mission finishes. The machine-readable report is written to
`/tmp/echorescue-portfolio-demo.json`; set `ECHORESCUE_PORTFOLIO_REPORT` to
choose another path.

## Verified run

A complete graphical run passed every acceptance check (arm, takeoff, outbound
doorway crossing, far-side goal region entry, return doorway crossing, land,
and disarm) with no prohibited obstacle contact and a fully clean process/port
teardown. Evidence is preserved at
`artifacts/v0.15.0-portfolio-demo-graphical-20260926-131447.json`.

The demo starts and owns its Gazebo and SITL processes. Do not run it
concurrently with another simulator using TCP 5760 / UDP 9002 / UDP 14550 on
the same machine; the report checks that these resources are released after
the run.

## Best screenshot moment

Using the evaluator's own timestamps from the verified run:

- **~t ≈ 40–42 s** (simulation time): the drone crosses the doorway outbound,
  framed between the two rooms — the clearest single-frame proof of real
  indoor doorway navigation.
- **~t ≈ 45–58 s**: the drone is in the far room near the planner's goal
  region — a strong "searching the far side of the structure" composition.

Either window works well with the fixed elevated oblique camera in
`config/portfolio-gui.config`. Avoid the first ~30 s (climb/hover near the
launch pad) and the final ~15 s (descent/landing), where the drone is close to
the floor and less visually prominent.
