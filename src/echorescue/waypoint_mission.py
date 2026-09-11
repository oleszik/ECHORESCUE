"""ROS-independent v0.14.4 continuous waypoint mission control.

MAVLink ``SET_POSITION_TARGET_LOCAL_NED`` is a setpoint message, not a command
microservice request.  Target acceptance therefore uses transmission success
and strictly post-transmission vehicle telemetry, never ``COMMAND_ACK``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
from math import hypot, isfinite, radians, sqrt
from typing import Any

from echorescue.continuous_vehicle_state import enu_to_ned, heading_enu_deg_to_ned_deg
from echorescue.flight_mission import (
    CommandRequest,
    FlightMissionController,
    MissionConfig,
    MissionPhase,
)
from echorescue.mavlink_telemetry import TelemetryHealth


POSITION_ONLY_TYPE_MASK = 0x0DF8
POSITION_AND_YAW_TYPE_MASK = 0x09F8
MAV_FRAME_LOCAL_NED = 1


@dataclass(frozen=True, slots=True)
class EnuTarget:
    """One immutable mission-local absolute ENU target."""

    target_id: str
    east_m: float
    north_m: float
    up_m: float
    heading_enu_deg: float | None = None

    def __post_init__(self) -> None:
        if not self.target_id or not self.target_id.isascii():
            raise ValueError("target_id must be non-empty ASCII")
        values = (self.east_m, self.north_m, self.up_m)
        if not all(isfinite(value) for value in values):
            raise ValueError("target coordinates must be finite")
        if self.heading_enu_deg is not None and not isfinite(self.heading_enu_deg):
            raise ValueError("target heading must be finite when present")


@dataclass(frozen=True, slots=True)
class NedSetpoint:
    target_id: str
    north_m: float
    east_m: float
    down_m: float
    yaw_rad: float
    type_mask: int
    coordinate_frame: int = MAV_FRAME_LOCAL_NED


def enu_heading_to_ned_yaw(heading_enu_deg: float) -> float:
    """Convert CCW-from-East ENU heading to clockwise-from-North NED yaw."""
    return radians(heading_enu_deg_to_ned_deg(heading_enu_deg))


def enu_target_to_ned(target: EnuTarget) -> NedSetpoint:
    heading = target.heading_enu_deg
    north, east, down = enu_to_ned((target.east_m, target.north_m, target.up_m))
    return NedSetpoint(
        target_id=target.target_id,
        north_m=north,
        east_m=east,
        down_m=down,
        yaw_rad=0.0 if heading is None else enu_heading_to_ned_yaw(heading),
        type_mask=POSITION_ONLY_TYPE_MASK if heading is None else POSITION_AND_YAW_TYPE_MASK,
    )


def target_errors(target: EnuTarget, east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    horizontal = hypot(target.east_m - east_m, target.north_m - north_m)
    vertical = abs(target.up_m - up_m)
    return horizontal, vertical, sqrt(horizontal * horizontal + vertical * vertical)


class NavigationPhase(str, Enum):
    FLIGHT_SEQUENCE = "flight_sequence"
    SEND_TARGET = "send_target"
    WAIT_TRANSMISSION = "wait_transmission"
    TRACK_TARGET = "track_target"
    LANDING = "landing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class RelativeTarget:
    target_id: str
    east_offset_m: float
    north_offset_m: float
    altitude_above_launch_m: float
    heading_enu_deg: float | None = None

    def __post_init__(self) -> None:
        values = (self.east_offset_m, self.north_offset_m, self.altitude_above_launch_m)
        if not self.target_id or not all(isfinite(value) for value in values):
            raise ValueError("relative target values must be finite and target_id must be non-empty")


@dataclass(frozen=True, slots=True)
class WaypointMissionConfig:
    targets: tuple[RelativeTarget, ...]
    takeoff_altitude_m: float = 2.0
    horizontal_tolerance_m: float = 0.45
    vertical_tolerance_m: float = 0.35
    settling_time_s: float = 2.0
    target_timeout_s: float = 30.0
    progress_timeout_s: float = 8.0
    progress_epsilon_m: float = 0.15
    mission_timeout_s: float = 150.0
    geofence_horizontal_radius_m: float = 8.0
    geofence_min_altitude_m: float = -0.25
    geofence_max_altitude_m: float = 4.0
    ready_timeout_s: float = 45.0
    preflight_hold_s: float = 3.0
    transition_timeout_s: float = 15.0
    takeoff_timeout_s: float = 30.0
    landing_timeout_s: float = 35.0
    cleanup_timeout_s: float = 35.0

    def __post_init__(self) -> None:
        positive = (
            self.takeoff_altitude_m, self.horizontal_tolerance_m,
            self.vertical_tolerance_m, self.settling_time_s,
            self.target_timeout_s, self.progress_timeout_s,
            self.progress_epsilon_m, self.mission_timeout_s,
            self.geofence_horizontal_radius_m, self.ready_timeout_s,
            self.preflight_hold_s, self.transition_timeout_s,
            self.takeoff_timeout_s, self.landing_timeout_s,
            self.cleanup_timeout_s,
        )
        if not self.targets or not all(isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("waypoint mission durations, tolerances, limits, and targets must be positive")
        ids = [target.target_id for target in self.targets]
        if len(ids) != len(set(ids)):
            raise ValueError("target IDs must be unique")
        if self.geofence_min_altitude_m >= self.geofence_max_altitude_m:
            raise ValueError("geofence altitude bounds are invalid")
        if self.takeoff_altitude_m > self.geofence_max_altitude_m:
            raise ValueError("takeoff altitude exceeds the geofence")
        if self.ready_timeout_s <= self.preflight_hold_s:
            raise ValueError("ready timeout must exceed preflight hold")


@dataclass(frozen=True, slots=True)
class TargetRequest:
    session_id: str
    target: EnuTarget
    ned: NedSetpoint


@dataclass(frozen=True, slots=True)
class WaypointMissionEvent:
    sequence: int
    monotonic_ns: int
    session_id: str
    phase: str
    event: str
    target_id: str
    detail: str


class WaypointMissionController:
    """Closed-loop one-vehicle waypoint controller with bounded LAND recovery."""

    def __init__(self, config: WaypointMissionConfig, started_monotonic_ns: int) -> None:
        self.config = config
        flight_config = MissionConfig(
            target_altitude_enu_m=config.takeoff_altitude_m,
            altitude_tolerance_m=config.vertical_tolerance_m,
            hover_duration_s=config.settling_time_s,
            preflight_hold_s=config.preflight_hold_s,
            ready_timeout_s=config.ready_timeout_s,
            transition_timeout_s=config.transition_timeout_s,
            takeoff_timeout_s=config.takeoff_timeout_s,
            landing_timeout_s=config.landing_timeout_s,
            cleanup_timeout_s=config.cleanup_timeout_s,
            auto_land_after_hover=False,
        )
        self.flight = FlightMissionController(flight_config, started_monotonic_ns)
        self.phase = NavigationPhase.FLIGHT_SEQUENCE
        self.started_monotonic_ns = started_monotonic_ns
        self.phase_started_ns = started_monotonic_ns
        self.launch_enu: tuple[float, float, float] | None = None
        self._position: tuple[float, float, float] | None = None
        self._position_session = ""
        self._position_source_ms: int | None = None
        self._position_receipt_ns = 0
        self._previous_position_receipt_ns = 0
        self._target_index = 0
        self._active_target: EnuTarget | None = None
        self._transmission_pending = False
        self._transmit_ns: int | None = None
        self._transmit_source_ms: int | None = None
        self._first_post_ns: int | None = None
        self._first_post_source_ms: int | None = None
        self._tracking_started_ns: int | None = None
        self._arrival_started_ns: int | None = None
        self._last_progress_ns: int | None = None
        self._first_progress_ns: int | None = None
        self._initial_error_m: float | None = None
        self._best_error_m: float | None = None
        self._progress_reference_error_m: float | None = None
        self._measured_at_best: tuple[float, float, float] | None = None
        self._failure_reason: str | None = None
        self.events: list[WaypointMissionEvent] = []
        self.targets: list[dict[str, Any]] = []
        self.geofence_checks: list[dict[str, Any]] = []
        self.maximum_altitude_enu_m: float | None = None
        self.maximum_horizontal_displacement_m = 0.0
        self.maximum_telemetry_age_s = 0.0
        self.maximum_telemetry_gap_s = 0.0
        self._record(started_monotonic_ns, "mission_started", "waiting for launch-local telemetry")

    @property
    def terminal(self) -> bool:
        return self.phase in (NavigationPhase.SUCCEEDED, NavigationPhase.FAILED)

    @property
    def succeeded(self) -> bool:
        return self.phase is NavigationPhase.SUCCEEDED

    @property
    def session_id(self) -> str:
        return self.flight.session_id

    @property
    def active_target(self) -> EnuTarget | None:
        return self._active_target

    @property
    def latest_source_time_boot_ms(self) -> int | None:
        return self._position_source_ms

    def _record(self, now_ns: int, event: str, detail: str, target_id: str = "") -> None:
        self.events.append(WaypointMissionEvent(
            len(self.events) + 1, now_ns, self.session_id, self.phase.value,
            event, target_id, detail,
        ))

    def _set_phase(self, phase: NavigationPhase, now_ns: int, detail: str) -> None:
        self.phase = phase
        self.phase_started_ns = now_ns
        self._record(now_ns, "phase", detail, self._active_target.target_id if self._active_target else "")

    def observe_status(
        self, *, session_id: str, health: TelemetryHealth,
        telemetry_age_s: float, now_ns: int,
    ) -> None:
        old_session = self.session_id
        if (
            session_id == old_session
            and health in (TelemetryHealth.STALE, TelemetryHealth.DISCONNECTED)
            and self.phase in (
                NavigationPhase.SEND_TARGET,
                NavigationPhase.WAIT_TRANSMISSION,
                NavigationPhase.TRACK_TARGET,
            )
        ):
            self._abort(now_ns, f"telemetry became {health.value} during navigation")
        self.flight.observe_status(
            session_id=session_id, health=health,
            telemetry_age_s=telemetry_age_s, now_ns=now_ns,
        )
        if telemetry_age_s >= 0.0:
            self.maximum_telemetry_age_s = max(self.maximum_telemetry_age_s, telemetry_age_s)
        self._after_session_observation(old_session, now_ns)
        self._sync_flight_terminal(now_ns)

    def observe_heartbeat(self, *, session_id: str, armed: bool, flight_mode: str, now_ns: int) -> None:
        old_session = self.session_id
        self.flight.observe_heartbeat(
            session_id=session_id, armed=armed, flight_mode=flight_mode, now_ns=now_ns,
        )
        self._after_session_observation(old_session, now_ns)
        if self.phase in (NavigationPhase.SEND_TARGET, NavigationPhase.WAIT_TRANSMISSION, NavigationPhase.TRACK_TARGET):
            if armed and flight_mode != "GUIDED":
                self._abort(now_ns, f"unexpected flight mode during navigation: {flight_mode}")
            elif not armed and self.launch_enu is not None and (
                self.flight.landed is False
                or (self._position is not None and self._position[2] > self.launch_enu[2] + self.config.vertical_tolerance_m)
            ):
                self._abort(now_ns, "unexpected disarm while airborne")
        self._sync_flight_terminal(now_ns)

    def observe_landed(self, *, session_id: str, landed: bool | None, now_ns: int) -> None:
        if self.session_id and session_id != self.session_id:
            self._record(now_ns, "telemetry_ignored", "landed state belongs to a non-active session")
            return
        old_session = self.session_id
        self.flight.observe_landed(session_id=session_id, landed=landed, now_ns=now_ns)
        self._after_session_observation(old_session, now_ns)
        if landed is True and self.phase in (
            NavigationPhase.SEND_TARGET, NavigationPhase.WAIT_TRANSMISSION, NavigationPhase.TRACK_TARGET,
        ):
            self._abort(now_ns, "unexpected landing during navigation")
        self._sync_flight_terminal(now_ns)

    def observe_position(
        self, *, session_id: str, east_m: float, north_m: float, up_m: float,
        source_time_boot_ms: int, now_ns: int,
    ) -> None:
        if not all(isfinite(value) for value in (east_m, north_m, up_m)) or source_time_boot_ms < 0:
            return
        if self.session_id and session_id != self.session_id:
            self._record(now_ns, "telemetry_ignored", "local position belongs to a non-active session")
            return
        old_session = self.session_id
        self.flight.observe_position(session_id=session_id, altitude_enu_m=up_m, now_ns=now_ns)
        self._after_session_observation(old_session, now_ns)
        if session_id != self.session_id:
            return
        if self._position_session == session_id and self._position_source_ms is not None:
            if source_time_boot_ms < self._position_source_ms:
                self._abort(now_ns, "vehicle-time regression in local-position telemetry")
                return
            if source_time_boot_ms == self._position_source_ms or now_ns <= self._position_receipt_ns:
                self._record(now_ns, "telemetry_ignored", "duplicate or out-of-order local position")
                return
        if self._previous_position_receipt_ns:
            gap_s = (now_ns - self._previous_position_receipt_ns) / 1e9
            self.maximum_telemetry_gap_s = max(self.maximum_telemetry_gap_s, gap_s)
        self._previous_position_receipt_ns = now_ns
        self._position = (east_m, north_m, up_m)
        self._position_session = session_id
        self._position_source_ms = source_time_boot_ms
        self._position_receipt_ns = now_ns
        self.maximum_altitude_enu_m = up_m if self.maximum_altitude_enu_m is None else max(self.maximum_altitude_enu_m, up_m)
        if self.launch_enu is None and self.flight.armed is False and self.flight.health is TelemetryHealth.CONNECTED:
            self.launch_enu = self._position
            self._record(now_ns, "launch_recorded", "valid active-session ENU launch position recorded")
        if self.launch_enu is not None:
            horizontal = hypot(east_m - self.launch_enu[0], north_m - self.launch_enu[1])
            self.maximum_horizontal_displacement_m = max(self.maximum_horizontal_displacement_m, horizontal)
            relative_up = up_m - self.launch_enu[2]
            if (
                self.phase is NavigationPhase.FLIGHT_SEQUENCE
                and self.flight.phase is MissionPhase.HOVER
                and horizontal > self.config.horizontal_tolerance_m
            ):
                # The inherited v0.14.3 hover clock already enforces continuous
                # altitude residence. Reset it while outside the launch-local
                # horizontal tolerance so takeoff settling is genuinely 3D.
                self.flight.hover_started_ns = now_ns
            if (
                horizontal > self.config.geofence_horizontal_radius_m
                or relative_up < self.config.geofence_min_altitude_m
                or relative_up > self.config.geofence_max_altitude_m
            ) and self.flight.armed is True:
                self._abort(now_ns, "observed vehicle state violated the configured ENU geofence")
                return
        if self.phase is NavigationPhase.TRACK_TARGET:
            self._track_target(now_ns, source_time_boot_ms)
        self._sync_flight_terminal(now_ns)

    def _after_session_observation(self, old_session: str, now_ns: int) -> None:
        if old_session and self.session_id != old_session:
            self._invalidate_target(now_ns, "MAVLink session changed; target correlation invalidated")
            self.launch_enu = None
            self._position = None
            self._position_session = ""
            self._position_source_ms = None
            self._position_receipt_ns = 0
            self._previous_position_receipt_ns = 0
            if not self.terminal:
                self._set_phase(NavigationPhase.LANDING, now_ns, "session reconnect entered bounded flight recovery")

    def acknowledge(self, *, session_id: str, command_id: int, result: int, now_ns: int) -> bool:
        accepted = self.flight.acknowledge(
            session_id=session_id, command_id=command_id, result=result, now_ns=now_ns,
        )
        self._sync_flight_terminal(now_ns)
        return accepted

    def tick(self, now_ns: int) -> CommandRequest | TargetRequest | None:
        if self.terminal:
            return None
        if (now_ns - self.started_monotonic_ns) / 1e9 > self.config.mission_timeout_s:
            self._abort(now_ns, "overall mission timeout")
        if self.phase in (NavigationPhase.FLIGHT_SEQUENCE, NavigationPhase.LANDING):
            request = self.flight.tick(now_ns)
            if self.flight.phase is MissionPhase.NAVIGATION_HOLD and self.phase is NavigationPhase.FLIGHT_SEQUENCE:
                if self.launch_enu is None or self._position_session != self.session_id:
                    self._abort(now_ns, "no active-session launch position available after takeoff")
                else:
                    self._build_next_target(now_ns)
            self._sync_flight_terminal(now_ns)
            return request
        if self.phase is NavigationPhase.SEND_TARGET:
            if not self._fresh_position:
                self._abort(now_ns, "cannot transmit target without fresh active-session telemetry")
                return self._recovery_tick(now_ns)
            assert self._active_target is not None
            if not self._target_inside_geofence(self._active_target, now_ns):
                self._abort(now_ns, f"target {self._active_target.target_id} violates the configured geofence")
                return self._recovery_tick(now_ns)
            self._transmission_pending = True
            self._set_phase(NavigationPhase.WAIT_TRANSMISSION, now_ns, "setpoint handed to MAVLink boundary")
            return TargetRequest(self.session_id, self._active_target, enu_target_to_ned(self._active_target))
        if self.phase is NavigationPhase.WAIT_TRANSMISSION:
            if (now_ns - self.phase_started_ns) / 1e9 > self.config.transition_timeout_s:
                self._abort(now_ns, "target transmission result timeout")
        elif self.phase is NavigationPhase.TRACK_TARGET:
            assert self._tracking_started_ns is not None
            elapsed_s = (now_ns - self._tracking_started_ns) / 1e9
            if elapsed_s > self.config.target_timeout_s:
                self._abort(now_ns, f"target {self._active_target_id} timeout")
            elif self._arrival_started_ns is None and self._last_progress_ns is not None and (
                now_ns - self._last_progress_ns
            ) / 1e9 > self.config.progress_timeout_s:
                self._abort(now_ns, f"no measurable progress toward target {self._active_target_id}")
        return self._recovery_tick(now_ns)

    def target_transmitted(self, *, session_id: str, success: bool, now_ns: int, detail: str = "") -> bool:
        if self.phase is not NavigationPhase.WAIT_TRANSMISSION or not self._transmission_pending:
            self._record(now_ns, "transmission_ignored", "no target transmission is pending")
            return False
        if session_id != self.session_id:
            self._record(now_ns, "transmission_ignored", "target transmission belongs to an old session")
            return False
        self._transmission_pending = False
        if not success:
            self._abort(now_ns, f"target transmission failed: {detail or 'transport rejected send'}")
            return True
        assert self._active_target is not None and self._position is not None
        self._transmit_ns = now_ns
        self._transmit_source_ms = self._position_source_ms
        self._tracking_started_ns = now_ns
        self._last_progress_ns = now_ns
        horizontal, vertical, total = target_errors(self._active_target, *self._position)
        self._initial_error_m = total
        self._best_error_m = total
        self._progress_reference_error_m = total
        self._measured_at_best = self._position
        record = self.targets[-1]
        record.update({
            "transmitted": True,
            "transmit_monotonic_ns": now_ns,
            "transmit_source_time_boot_ms": self._transmit_source_ms,
            "initial_horizontal_error_m": horizontal,
            "initial_vertical_error_m": vertical,
            "initial_error_m": total,
            "transmission_detail": detail,
        })
        self._record(now_ns, "target_transmitted", "SET_POSITION_TARGET_LOCAL_NED sent successfully", self._active_target.target_id)
        self._set_phase(NavigationPhase.TRACK_TARGET, now_ns, "awaiting strictly post-transmission vehicle telemetry")
        return True

    @property
    def _active_target_id(self) -> str:
        return self._active_target.target_id if self._active_target else ""

    @property
    def _fresh_position(self) -> bool:
        return (
            self.flight.health is TelemetryHealth.CONNECTED
            and self.flight.telemetry_age_s >= 0.0
            and self._position is not None
            and self._position_session == self.session_id
        )

    def _build_next_target(self, now_ns: int) -> None:
        assert self.launch_enu is not None
        if self._target_index >= len(self.config.targets):
            self.flight.request_land(now_ns)
            self._active_target = None
            self._set_phase(NavigationPhase.LANDING, now_ns, "all waypoint and return targets settled")
            return
        relative = self.config.targets[self._target_index]
        target = EnuTarget(
            relative.target_id,
            self.launch_enu[0] + relative.east_offset_m,
            self.launch_enu[1] + relative.north_offset_m,
            self.launch_enu[2] + relative.altitude_above_launch_m,
            relative.heading_enu_deg,
        )
        self._active_target = target
        ned = enu_target_to_ned(target)
        self.targets.append({
            "target_id": target.target_id,
            "session_id": self.session_id,
            "sequence": self._target_index + 1,
            "commanded_enu": asdict(target),
            "transmitted_ned": asdict(ned),
            "transmitted": False,
            "outcome": "pending",
            "failure_reason": None,
        })
        self._reset_tracking()
        self._set_phase(NavigationPhase.SEND_TARGET, now_ns, f"target {target.target_id} activated")

    def _track_target(self, now_ns: int, source_time_ms: int) -> None:
        if (
            self._active_target is None or self._transmit_ns is None
            or self._transmit_source_ms is None or now_ns <= self._transmit_ns
            or source_time_ms <= self._transmit_source_ms
        ):
            return
        assert self._position is not None
        horizontal, vertical, total = target_errors(self._active_target, *self._position)
        record = self.targets[-1]
        if self._first_post_ns is None:
            self._first_post_ns = now_ns
            self._first_post_source_ms = source_time_ms
            record["first_post_command_monotonic_ns"] = now_ns
            record["first_post_command_source_time_boot_ms"] = source_time_ms
            self._record(now_ns, "first_post_command_telemetry", "fresh vehicle-time-progressing state observed", self._active_target.target_id)
        assert self._best_error_m is not None
        if total < self._best_error_m:
            self._best_error_m = total
            self._measured_at_best = self._position
            assert self._progress_reference_error_m is not None
            if self._progress_reference_error_m - total >= self.config.progress_epsilon_m:
                self._progress_reference_error_m = total
                self._last_progress_ns = now_ns
                if self._first_progress_ns is None:
                    self._first_progress_ns = now_ns
                    record["time_to_first_progress_s"] = (now_ns - self._transmit_ns) / 1e9
                    self._record(now_ns, "target_progress", "measurable target-error decrease observed", self._active_target.target_id)
        inside = horizontal <= self.config.horizontal_tolerance_m and vertical <= self.config.vertical_tolerance_m
        if inside:
            if self._arrival_started_ns is None:
                self._arrival_started_ns = now_ns
                record["arrival_monotonic_ns"] = now_ns
                record["time_to_arrival_s"] = (now_ns - self._transmit_ns) / 1e9
                self._record(now_ns, "target_arrival", "horizontal and vertical tolerances entered", self._active_target.target_id)
            if (now_ns - self._arrival_started_ns) / 1e9 >= self.config.settling_time_s:
                record.update({
                    "outcome": "settled",
                    "settled_monotonic_ns": now_ns,
                    "settling_duration_s": (now_ns - self._arrival_started_ns) / 1e9,
                    "measured_enu": {
                        "east_m": self._position[0], "north_m": self._position[1], "up_m": self._position[2],
                    },
                    "horizontal_error_m": horizontal,
                    "vertical_error_m": vertical,
                    "total_error_m": total,
                    "best_error_m": self._best_error_m,
                    "measured_at_best_enu": self._pose_dict(self._measured_at_best),
                })
                self._record(now_ns, "target_settled", "continuous residence interval completed", self._active_target.target_id)
                self._target_index += 1
                self._build_next_target(now_ns)
        elif self._arrival_started_ns is not None:
            self._arrival_started_ns = None
            record.pop("arrival_monotonic_ns", None)
            record.pop("time_to_arrival_s", None)
            self._record(now_ns, "arrival_reset", "vehicle left target tolerance before settling", self._active_target.target_id)

    def _target_inside_geofence(self, target: EnuTarget, now_ns: int) -> bool:
        assert self.launch_enu is not None
        horizontal = hypot(target.east_m - self.launch_enu[0], target.north_m - self.launch_enu[1])
        relative_up = target.up_m - self.launch_enu[2]
        passed = (
            horizontal <= self.config.geofence_horizontal_radius_m
            and self.config.geofence_min_altitude_m <= relative_up <= self.config.geofence_max_altitude_m
        )
        self.geofence_checks.append({
            "target_id": target.target_id, "session_id": self.session_id,
            "horizontal_distance_m": horizontal, "relative_altitude_m": relative_up,
            "horizontal_limit_m": self.config.geofence_horizontal_radius_m,
            "minimum_altitude_m": self.config.geofence_min_altitude_m,
            "maximum_altitude_m": self.config.geofence_max_altitude_m,
            "passed": passed, "checked_monotonic_ns": now_ns,
        })
        return passed

    def _reset_tracking(self) -> None:
        self._transmission_pending = False
        self._transmit_ns = None
        self._transmit_source_ms = None
        self._first_post_ns = None
        self._first_post_source_ms = None
        self._tracking_started_ns = None
        self._arrival_started_ns = None
        self._last_progress_ns = None
        self._first_progress_ns = None
        self._initial_error_m = None
        self._best_error_m = None
        self._progress_reference_error_m = None
        self._measured_at_best = None

    def _invalidate_target(self, now_ns: int, reason: str) -> None:
        if self._active_target is not None and self.targets and self.targets[-1]["outcome"] == "pending":
            self.targets[-1]["outcome"] = "invalidated"
            self.targets[-1]["failure_reason"] = reason
            self._record(now_ns, "target_invalidated", reason, self._active_target.target_id)
        self._active_target = None
        self._reset_tracking()

    def _abort(self, now_ns: int, reason: str) -> None:
        if self.terminal or self.phase is NavigationPhase.LANDING:
            return
        self._failure_reason = reason
        if self._active_target is not None and self.targets and self.targets[-1]["outcome"] == "pending":
            self.targets[-1]["outcome"] = "failed"
            self.targets[-1]["failure_reason"] = reason
        self._record(now_ns, "mission_abort", reason, self._active_target_id)
        self._active_target = None
        self._reset_tracking()
        self.flight.cancel(now_ns, reason)
        self._set_phase(NavigationPhase.LANDING, now_ns, "waypoint plan stopped; bounded landing recovery active")
        self._sync_flight_terminal(now_ns)

    def cancel(self, now_ns: int, reason: str = "mission process interrupted") -> None:
        self._abort(now_ns, reason)

    def _recovery_tick(self, now_ns: int) -> CommandRequest | None:
        if self.phase is NavigationPhase.LANDING:
            request = self.flight.tick(now_ns)
            self._sync_flight_terminal(now_ns)
            return request
        return None

    def _sync_flight_terminal(self, now_ns: int) -> None:
        if not self.flight.terminal or self.terminal:
            return
        if self.flight.succeeded and self._target_index == len(self.config.targets) and self._failure_reason is None:
            self._set_phase(NavigationPhase.SUCCEEDED, now_ns, "LAND ACK, ON_GROUND, and disarmed telemetry verified")
        else:
            if self._failure_reason is None:
                self._failure_reason = self.flight.report().get("failure_reason")
            self._set_phase(NavigationPhase.FAILED, now_ns, self._failure_reason or "flight sequence failed")

    @staticmethod
    def _pose_dict(pose: tuple[float, float, float] | None) -> dict[str, float] | None:
        if pose is None:
            return None
        return {"east_m": pose[0], "north_m": pose[1], "up_m": pose[2]}

    def report(self) -> dict[str, Any]:
        final_distance = None
        if self.launch_enu is not None and self._position is not None:
            final_distance = sqrt(sum((value - origin) ** 2 for value, origin in zip(self._position, self.launch_enu)))
        flight_report = self.flight.report()
        command_results = {
            kind: next(
                (item["ack_result"] for item in flight_report["commands"] if item["kind"] == kind),
                None,
            )
            for kind in ("guided", "arm", "takeoff", "land")
        }
        return {
            "schema_version": "echorescue-continuous-waypoint-mission/1.0",
            "milestone": "v0.14.4",
            "status": "PASS" if self.succeeded else ("FAIL" if self.terminal else "RUNNING"),
            "phase": self.phase.value,
            "failure_reason": self._failure_reason,
            "session_id": self.session_id,
            "launch_enu": self._pose_dict(self.launch_enu),
            "targets": self.targets,
            "commands": flight_report["commands"],
            "command_ack_results": command_results,
            "disarm": {
                "command_issued": False,
                "ack_result": None,
                "telemetry_verified": self.flight.armed is False,
                "detail": "ArduPilot disarmed after LAND; EchoRescue issued no disarm command",
            },
            "flight_events": flight_report["events"],
            "mission_events": [asdict(event) for event in self.events],
            "geofence_checks": self.geofence_checks,
            "maximum_altitude_enu_m": self.maximum_altitude_enu_m,
            "maximum_horizontal_displacement_m": self.maximum_horizontal_displacement_m,
            "maximum_telemetry_age_s": self.maximum_telemetry_age_s,
            "maximum_telemetry_gap_s": self.maximum_telemetry_gap_s,
            "final_position_enu": self._pose_dict(self._position),
            "final_distance_from_launch_m": final_distance,
            "final_landed": self.flight.landed,
            "final_armed": self.flight.armed,
        }


def serialize_waypoint_report(report: dict[str, Any]) -> str:
    return json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
