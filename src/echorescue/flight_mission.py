"""ROS-independent v0.14.3 arm/takeoff/hover/land mission logic.

The controller deliberately separates MAVLink command acknowledgements from
telemetry evidence.  A successful ACK never changes vehicle state by itself;
each phase also requires a later heartbeat, local-position, or landed-state
observation from the flight controller.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
from math import isfinite
from typing import Any

from echorescue.mavlink_telemetry import TelemetryHealth


MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_EXTENDED_SYS_STATE = 245
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5


def validate_simulation_endpoint(endpoint: str) -> str:
    """Reject serial, UDP, and non-loopback command transports."""
    prefix = "tcp:127.0.0.1:"
    if not endpoint.startswith(prefix) or not endpoint[len(prefix):].isdigit():
        raise ValueError("v0.14.3 commands require a loopback TCP SITL endpoint")
    return endpoint


class MissionPhase(str, Enum):
    WAIT_READY = "wait_ready"
    SEND_EXTENDED_STATE_STREAM = "send_extended_state_stream"
    WAIT_EXTENDED_STATE_STREAM = "wait_extended_state_stream"
    SEND_GUIDED = "send_guided"
    WAIT_GUIDED = "wait_guided"
    SEND_ARM = "send_arm"
    WAIT_ARMED = "wait_armed"
    SEND_TAKEOFF = "send_takeoff"
    WAIT_ALTITUDE = "wait_altitude"
    HOVER = "hover"
    SEND_LAND = "send_land"
    WAIT_LANDED_DISARMED = "wait_landed_disarmed"
    RECOVERY_WAIT_TELEMETRY = "recovery_wait_telemetry"
    RECOVERY_SEND_LAND = "recovery_send_land"
    RECOVERY_WAIT_LANDED_DISARMED = "recovery_wait_landed_disarmed"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class CommandKind(str, Enum):
    EXTENDED_STATE_STREAM = "extended_state_stream"
    GUIDED = "guided"
    ARM = "arm"
    TAKEOFF = "takeoff"
    LAND = "land"


@dataclass(frozen=True, slots=True)
class MissionConfig:
    target_altitude_enu_m: float = 2.0
    altitude_tolerance_m: float = 0.35
    hover_duration_s: float = 3.0
    preflight_hold_s: float = 30.0
    ready_timeout_s: float = 45.0
    transition_timeout_s: float = 15.0
    takeoff_timeout_s: float = 30.0
    landing_timeout_s: float = 30.0
    cleanup_timeout_s: float = 30.0

    def __post_init__(self) -> None:
        positive = (
            self.target_altitude_enu_m,
            self.altitude_tolerance_m,
            self.hover_duration_s,
            self.preflight_hold_s,
            self.ready_timeout_s,
            self.transition_timeout_s,
            self.takeoff_timeout_s,
            self.landing_timeout_s,
            self.cleanup_timeout_s,
        )
        if not all(isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("mission configuration values must be finite and positive")
        if self.altitude_tolerance_m >= self.target_altitude_enu_m:
            raise ValueError("altitude_tolerance_m must be below target_altitude_enu_m")
        if self.ready_timeout_s <= self.preflight_hold_s:
            raise ValueError("ready_timeout_s must exceed preflight_hold_s")


@dataclass(frozen=True, slots=True)
class CommandRequest:
    kind: CommandKind
    command_id: int
    target_altitude_enu_m: float | None = None
    recovery: bool = False


@dataclass(frozen=True, slots=True)
class MissionEvent:
    monotonic_ns: int
    phase: str
    event: str
    detail: str


class FlightMissionController:
    """Deterministic event-driven controller for one simulation vehicle."""

    def __init__(self, config: MissionConfig, started_monotonic_ns: int) -> None:
        if started_monotonic_ns < 0:
            raise ValueError("started_monotonic_ns must be non-negative")
        self.config = config
        self.phase = MissionPhase.WAIT_READY
        self.started_monotonic_ns = started_monotonic_ns
        self.phase_started_ns = started_monotonic_ns
        self.session_id = ""
        self.health = TelemetryHealth.DISCONNECTED
        self.telemetry_age_s = -1.0
        self.armed: bool | None = None
        self.flight_mode: str | None = None
        self.altitude_enu_m: float | None = None
        self.landed: bool | None = None
        self.last_heartbeat_ns = 0
        self.last_position_ns = 0
        self.last_landed_ns = 0
        self.hover_started_ns: int | None = None
        self._ready_since_ns: int | None = None
        self._pending: CommandRequest | None = None
        self._pending_session_id = ""
        self._pending_issued_ns = 0
        self._pending_acknowledged = False
        self._pending_ack_ns = 0
        self._failure_reason: str | None = None
        self._cleanup_reason: str | None = None
        self._cleanup_started_ns = 0
        self._land_acknowledged = False
        self.events: list[MissionEvent] = []
        self.command_history: list[dict[str, Any]] = []
        self._record(started_monotonic_ns, "mission_started", "waiting for fresh disarmed telemetry")

    @property
    def terminal(self) -> bool:
        return self.phase in (MissionPhase.SUCCEEDED, MissionPhase.FAILED)

    @property
    def succeeded(self) -> bool:
        return self.phase is MissionPhase.SUCCEEDED

    @property
    def pending_command(self) -> CommandRequest | None:
        return self._pending

    def _record(self, now_ns: int, event: str, detail: str) -> None:
        self.events.append(MissionEvent(now_ns, self.phase.value, event, detail))

    def _set_phase(self, phase: MissionPhase, now_ns: int, detail: str) -> None:
        self.phase = phase
        self.phase_started_ns = now_ns
        self._record(now_ns, "phase", detail)

    def observe_status(
        self,
        *,
        session_id: str,
        health: TelemetryHealth,
        telemetry_age_s: float,
        now_ns: int,
    ) -> None:
        previous_session = self.session_id
        self.health = health
        self.telemetry_age_s = telemetry_age_s
        if session_id and previous_session and session_id != previous_session and not self.terminal:
            self.session_id = session_id
            self._abort(now_ns, f"MAVLink reconnect changed session {previous_session} -> {session_id}")
            return
        if session_id:
            self.session_id = session_id
        if health in (TelemetryHealth.STALE, TelemetryHealth.DISCONNECTED) and self.phase not in (
            MissionPhase.WAIT_READY,
            MissionPhase.RECOVERY_WAIT_TELEMETRY,
            MissionPhase.RECOVERY_SEND_LAND,
            MissionPhase.RECOVERY_WAIT_LANDED_DISARMED,
            MissionPhase.SUCCEEDED,
            MissionPhase.FAILED,
        ):
            self._abort(now_ns, f"telemetry became {health.value}")

    def observe_heartbeat(self, *, session_id: str, armed: bool, flight_mode: str, now_ns: int) -> None:
        self._observe_session(session_id, now_ns)
        self.armed = armed
        self.flight_mode = flight_mode
        self.last_heartbeat_ns = now_ns

    def observe_position(self, *, session_id: str, altitude_enu_m: float, now_ns: int) -> None:
        if not isfinite(altitude_enu_m):
            return
        self._observe_session(session_id, now_ns)
        self.altitude_enu_m = altitude_enu_m
        self.last_position_ns = now_ns

    def observe_landed(self, *, session_id: str, landed: bool | None, now_ns: int) -> None:
        self._observe_session(session_id, now_ns)
        self.landed = landed
        self.last_landed_ns = now_ns

    def _observe_session(self, session_id: str, now_ns: int) -> None:
        if not session_id:
            return
        if self.session_id and session_id != self.session_id and not self.terminal:
            previous = self.session_id
            self.session_id = session_id
            self._abort(now_ns, f"MAVLink reconnect changed session {previous} -> {session_id}")
        else:
            self.session_id = session_id

    def acknowledge(
        self,
        *,
        session_id: str,
        command_id: int,
        result: int,
        now_ns: int,
    ) -> bool:
        if self._pending is None or command_id != self._pending.command_id:
            self._record(now_ns, "ack_ignored", f"unexpected COMMAND_ACK command={command_id} result={result}")
            return False
        if session_id != self._pending_session_id:
            self._record(
                now_ns,
                "ack_ignored",
                f"COMMAND_ACK session {session_id!r} does not match pending session {self._pending_session_id!r}",
            )
            return False
        if now_ns < self._pending_issued_ns:
            self._record(now_ns, "ack_ignored", "COMMAND_ACK predates the pending command")
            return False
        if result == MAV_RESULT_IN_PROGRESS:
            self._record(now_ns, "ack_progress", f"{self._pending.kind.value} is in progress")
            return True
        if result != MAV_RESULT_ACCEPTED:
            kind = self._pending.kind.value
            self.command_history[-1]["ack_result"] = result
            self.command_history[-1]["ack_monotonic_ns"] = now_ns
            self._pending = None
            self._abort(now_ns, f"{kind} command rejected with MAV_RESULT={result}")
            return True
        self._pending_acknowledged = True
        self._pending_ack_ns = now_ns
        self.command_history[-1]["ack_result"] = result
        self.command_history[-1]["ack_monotonic_ns"] = now_ns
        if self._pending.kind is CommandKind.LAND:
            self._land_acknowledged = True
        self._record(now_ns, "command_ack", f"{self._pending.kind.value} accepted")
        return True

    def tick(self, now_ns: int) -> CommandRequest | None:
        if now_ns < self.phase_started_ns:
            raise ValueError("now_ns must not precede the current phase")
        if self.terminal:
            return None
        if self.phase is MissionPhase.WAIT_READY:
            if self._fresh and self.armed is False and self.altitude_enu_m is not None:
                if self._ready_since_ns is None:
                    self._ready_since_ns = now_ns
                elif (now_ns - self._ready_since_ns) / 1e9 >= self.config.preflight_hold_s:
                    self._set_phase(
                        MissionPhase.SEND_EXTENDED_STATE_STREAM,
                        now_ns,
                        "fresh disarmed telemetry remained stable through preflight hold",
                    )
            else:
                self._ready_since_ns = None
            if self.phase is MissionPhase.WAIT_READY and self._elapsed_s(now_ns) > self.config.ready_timeout_s:
                self._fail(now_ns, "timed out waiting for fresh disarmed telemetry")
        elif self.phase is MissionPhase.SEND_EXTENDED_STATE_STREAM:
            return self._issue(
                CommandKind.EXTENDED_STATE_STREAM,
                MAV_CMD_SET_MESSAGE_INTERVAL,
                now_ns,
            )
        elif self.phase is MissionPhase.WAIT_EXTENDED_STATE_STREAM:
            new_landed_state = self.last_landed_ns > self._pending_ack_ns
            if self._pending_acknowledged and self.landed is True and new_landed_state:
                self._mark_transition("extended_state.landed=true", self.last_landed_ns)
                self._clear_pending()
                self._set_phase(MissionPhase.SEND_GUIDED, now_ns, "extended-state ACK and telemetry observed")
            elif self._pending_acknowledged and self.landed is False and new_landed_state:
                self._abort(now_ns, "initial extended state reported vehicle not landed")
            else:
                self._transition_timeout(now_ns, self.config.transition_timeout_s, "extended-state stream")
        elif self.phase is MissionPhase.SEND_GUIDED:
            return self._issue(CommandKind.GUIDED, MAV_CMD_DO_SET_MODE, now_ns)
        elif self.phase is MissionPhase.WAIT_GUIDED:
            if self._pending_acknowledged and self.flight_mode == "GUIDED" and self.last_heartbeat_ns > self._pending_ack_ns:
                self._mark_transition("heartbeat.flight_mode=GUIDED", self.last_heartbeat_ns)
                self._clear_pending()
                self._set_phase(MissionPhase.SEND_ARM, now_ns, "GUIDED ACK and heartbeat mode observed")
            else:
                self._transition_timeout(now_ns, self.config.transition_timeout_s, "GUIDED")
        elif self.phase is MissionPhase.SEND_ARM:
            return self._issue(CommandKind.ARM, MAV_CMD_COMPONENT_ARM_DISARM, now_ns)
        elif self.phase is MissionPhase.WAIT_ARMED:
            if self._pending_acknowledged and self.armed is True and self.last_heartbeat_ns > self._pending_ack_ns:
                self._mark_transition("heartbeat.armed=true", self.last_heartbeat_ns)
                self._clear_pending()
                self._set_phase(MissionPhase.SEND_TAKEOFF, now_ns, "arm ACK and armed heartbeat observed")
            else:
                self._transition_timeout(now_ns, self.config.transition_timeout_s, "arming")
        elif self.phase is MissionPhase.SEND_TAKEOFF:
            return self._issue(
                CommandKind.TAKEOFF,
                MAV_CMD_NAV_TAKEOFF,
                now_ns,
                target_altitude_enu_m=self.config.target_altitude_enu_m,
            )
        elif self.phase is MissionPhase.WAIT_ALTITUDE:
            altitude_reached = (
                self.altitude_enu_m is not None
                and abs(self.altitude_enu_m - self.config.target_altitude_enu_m) <= self.config.altitude_tolerance_m
                and self.last_position_ns > self._pending_ack_ns
            )
            if self._pending_acknowledged and altitude_reached:
                self._mark_transition(
                    "local_position_enu.z_in_target_band",
                    self.last_position_ns,
                )
                self._clear_pending()
                self.hover_started_ns = now_ns
                self._set_phase(MissionPhase.HOVER, now_ns, "takeoff ACK and target ENU altitude observed")
            else:
                self._transition_timeout(now_ns, self.config.takeoff_timeout_s, "takeoff altitude")
        elif self.phase is MissionPhase.HOVER:
            if not self._fresh:
                self._abort(now_ns, "telemetry became stale during hover")
            elif self.altitude_enu_m is not None and abs(self.altitude_enu_m - self.config.target_altitude_enu_m) > self.config.altitude_tolerance_m:
                self._abort(now_ns, "vehicle left the configured hover altitude band")
            elif self.hover_started_ns is not None and (now_ns - self.hover_started_ns) / 1e9 >= self.config.hover_duration_s:
                self._set_phase(MissionPhase.SEND_LAND, now_ns, "timed hover completed")
        elif self.phase is MissionPhase.SEND_LAND:
            return self._issue(CommandKind.LAND, MAV_CMD_NAV_LAND, now_ns)
        elif self.phase is MissionPhase.WAIT_LANDED_DISARMED:
            if self._land_complete:
                self._mark_transition("extended_state.landed=true;heartbeat.armed=false", now_ns)
                self._clear_pending()
                self._set_phase(MissionPhase.SUCCEEDED, now_ns, "land ACK, landed state, and disarmed heartbeat observed")
            else:
                if self._elapsed_s(now_ns) > self.config.landing_timeout_s:
                    self._failure_reason = "timed out waiting for landing and disarm ACK and telemetry transition"
                    self._cleanup_reason = self._failure_reason
                    self._cleanup_started_ns = now_ns
                    self._set_phase(
                        MissionPhase.RECOVERY_WAIT_LANDED_DISARMED,
                        now_ns,
                        "LAND remains outstanding; bounded cleanup continues without reissuing it",
                    )
        elif self.phase is MissionPhase.RECOVERY_WAIT_TELEMETRY:
            fresh_heartbeat = self._fresh and self.last_heartbeat_ns > self._cleanup_started_ns
            if fresh_heartbeat and self.armed is False:
                self._fail(now_ns, f"{self._cleanup_reason}; fresh telemetry verified disarmed")
            elif fresh_heartbeat and self.armed is True:
                self._set_phase(MissionPhase.RECOVERY_SEND_LAND, now_ns, "fresh armed telemetry restored for cleanup")
            elif self._elapsed_s(now_ns) > self.config.cleanup_timeout_s:
                self._fail(now_ns, f"{self._cleanup_reason}; cleanup timed out without fresh telemetry")
        elif self.phase is MissionPhase.RECOVERY_SEND_LAND:
            return self._issue(CommandKind.LAND, MAV_CMD_NAV_LAND, now_ns, recovery=True)
        elif self.phase is MissionPhase.RECOVERY_WAIT_LANDED_DISARMED:
            if self._land_complete:
                self._mark_transition("extended_state.landed=true;heartbeat.armed=false", now_ns)
                self._clear_pending()
                self._fail(now_ns, f"{self._cleanup_reason}; recovery landing verified")
            elif self._elapsed_s(now_ns) > self.config.cleanup_timeout_s:
                self._fail(now_ns, f"{self._cleanup_reason}; recovery landing was not verified")
        return None

    @property
    def _fresh(self) -> bool:
        return self.health is TelemetryHealth.CONNECTED and self.telemetry_age_s >= 0.0

    @property
    def _land_complete(self) -> bool:
        return (
            self._land_acknowledged
            and self.landed is True
            and self.armed is False
            and self.last_landed_ns > self._pending_ack_ns
            and self.last_heartbeat_ns > self._pending_ack_ns
        )

    def _elapsed_s(self, now_ns: int) -> float:
        return (now_ns - self.phase_started_ns) / 1_000_000_000.0

    def _issue(
        self,
        kind: CommandKind,
        command_id: int,
        now_ns: int,
        *,
        target_altitude_enu_m: float | None = None,
        recovery: bool = False,
    ) -> CommandRequest:
        request = CommandRequest(kind, command_id, target_altitude_enu_m, recovery)
        self._pending = request
        self._pending_session_id = self.session_id
        self._pending_issued_ns = now_ns
        self._pending_acknowledged = False
        self._pending_ack_ns = 0
        if kind is CommandKind.LAND:
            self._land_acknowledged = False
        self.command_history.append({
            "kind": kind.value,
            "command_id": command_id,
            "session_id": self.session_id,
            "issued_monotonic_ns": now_ns,
            "ack_result": None,
            "ack_monotonic_ns": None,
            "telemetry_transition": None,
            "telemetry_transition_monotonic_ns": None,
            "recovery": recovery,
        })
        waiting = {
            CommandKind.EXTENDED_STATE_STREAM: MissionPhase.WAIT_EXTENDED_STATE_STREAM,
            CommandKind.GUIDED: MissionPhase.WAIT_GUIDED,
            CommandKind.ARM: MissionPhase.WAIT_ARMED,
            CommandKind.TAKEOFF: MissionPhase.WAIT_ALTITUDE,
            CommandKind.LAND: (
                MissionPhase.RECOVERY_WAIT_LANDED_DISARMED
                if recovery else MissionPhase.WAIT_LANDED_DISARMED
            ),
        }[kind]
        self._set_phase(waiting, now_ns, f"{kind.value} command issued; awaiting ACK and telemetry")
        return request

    def _clear_pending(self) -> None:
        self._pending = None
        self._pending_session_id = ""
        self._pending_acknowledged = False
        self._pending_ack_ns = 0

    def _mark_transition(self, name: str, now_ns: int) -> None:
        if self.command_history:
            self.command_history[-1]["telemetry_transition"] = name
            self.command_history[-1]["telemetry_transition_monotonic_ns"] = now_ns

    def _transition_timeout(self, now_ns: int, timeout_s: float, label: str) -> None:
        if self._elapsed_s(now_ns) > timeout_s:
            self._abort(now_ns, f"timed out waiting for {label} ACK and telemetry transition")

    def _abort(self, now_ns: int, reason: str) -> None:
        if self.terminal or self.phase in (
            MissionPhase.RECOVERY_WAIT_TELEMETRY,
            MissionPhase.RECOVERY_SEND_LAND,
            MissionPhase.RECOVERY_WAIT_LANDED_DISARMED,
        ):
            return
        self._failure_reason = reason
        self._cleanup_reason = reason
        self._cleanup_started_ns = now_ns
        may_be_armed = self.armed is True or self.phase in (
            MissionPhase.WAIT_ARMED,
            MissionPhase.SEND_TAKEOFF,
            MissionPhase.WAIT_ALTITUDE,
            MissionPhase.HOVER,
            MissionPhase.SEND_LAND,
            MissionPhase.WAIT_LANDED_DISARMED,
        )
        land_pending_in_session = (
            self._pending is not None
            and self._pending.kind is CommandKind.LAND
            and self._pending_session_id == self.session_id
        )
        land_already_issued_in_session = any(
            item["kind"] == CommandKind.LAND.value
            and item["session_id"] == self.session_id
            for item in self.command_history
        )
        if not land_pending_in_session:
            self._clear_pending()
        if may_be_armed:
            if land_already_issued_in_session:
                next_phase = MissionPhase.RECOVERY_WAIT_LANDED_DISARMED
                detail = (
                    f"mission aborted: {reason}; existing LAND remains outstanding "
                    "and will not be reissued in this session"
                )
            else:
                next_phase = (
                    MissionPhase.RECOVERY_SEND_LAND
                    if self._fresh and self.armed is True
                    else MissionPhase.RECOVERY_WAIT_TELEMETRY
                )
                detail = f"mission aborted: {reason}; cleanup required"
            self._set_phase(next_phase, now_ns, detail)
        else:
            self._fail(now_ns, reason)

    def cancel(self, now_ns: int, reason: str = "mission cancelled") -> None:
        """Abort deliberately while preserving the normal recovery policy."""
        self._abort(now_ns, reason)

    def _fail(self, now_ns: int, reason: str) -> None:
        self._failure_reason = reason
        self._clear_pending()
        self._set_phase(MissionPhase.FAILED, now_ns, reason)

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": "echorescue-arm-takeoff-hover-land-mission/1.0",
            "milestone": "v0.14.3",
            "status": "PASS" if self.succeeded else ("FAIL" if self.terminal else "RUNNING"),
            "phase": self.phase.value,
            "failure_reason": self._failure_reason,
            "session_id": self.session_id,
            "target_altitude_enu_m": self.config.target_altitude_enu_m,
            "final_altitude_enu_m": self.altitude_enu_m,
            "final_armed": self.armed,
            "final_landed": self.landed,
            "commands": list(self.command_history),
            "events": [asdict(event) for event in self.events],
        }


def serialize_mission_report(report: dict[str, Any]) -> str:
    return json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
