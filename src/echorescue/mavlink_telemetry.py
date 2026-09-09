"""ROS-independent receive-only MAVLink telemetry contracts and state.

The module accepts already-decoded MAVLink fields.  It deliberately imports
neither pymavlink nor ROS so parsing and connection semantics remain directly
testable and the deterministic simulator stays independent of both stacks.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
from math import isfinite
from typing import Any, Mapping


AUTOPILOT_FRAME = "mavlink/autopilot"
LOCAL_NED_FRAME = "mavlink/local_ned"
GLOBAL_WGS84_FRAME = "wgs84"
ATTITUDE_NED_FRD_FRAME = "mavlink/ned_frd"
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128


class TelemetryHealth(str, Enum):
    CONNECTED = "connected"
    DEGRADED = "degraded"
    STALE = "stale"
    DISCONNECTED = "disconnected"


COPTER_MODES: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}


def _finite(value: Any, field: str) -> float:
    converted = float(value)
    if not isfinite(converted):
        raise ValueError(f"{field} must be finite")
    return converted


def _non_negative_int(value: Any, field: str) -> int:
    converted = int(value)
    if converted < 0:
        raise ValueError(f"{field} must be non-negative")
    return converted


@dataclass(frozen=True, slots=True)
class HeartbeatTelemetry:
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    receipt_monotonic_ns: int
    armed: bool
    flight_mode: str
    frame_id: str = AUTOPILOT_FRAME


@dataclass(frozen=True, slots=True)
class LocalPositionNedTelemetry:
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    source_time_boot_ms: int
    receipt_monotonic_ns: int
    north_m: float
    east_m: float
    down_m: float
    velocity_north_m_s: float
    velocity_east_m_s: float
    velocity_down_m_s: float
    frame_id: str = LOCAL_NED_FRAME


@dataclass(frozen=True, slots=True)
class GlobalPositionTelemetry:
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    source_time_boot_ms: int
    receipt_monotonic_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_m_msl: float
    relative_altitude_m: float
    velocity_north_m_s: float
    velocity_east_m_s: float
    velocity_down_m_s: float
    heading_deg: float | None
    frame_id: str = GLOBAL_WGS84_FRAME


@dataclass(frozen=True, slots=True)
class AttitudeNedFrdTelemetry:
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    source_time_boot_ms: int
    receipt_monotonic_ns: int
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    frame_id: str = ATTITUDE_NED_FRD_FRAME


@dataclass(frozen=True, slots=True)
class LandedStateTelemetry:
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    receipt_monotonic_ns: int
    landed: bool | None
    landed_state: str
    frame_id: str = AUTOPILOT_FRAME


@dataclass(frozen=True, slots=True)
class TelemetryStatus:
    session_id: str
    sequence: int
    health: TelemetryHealth
    connected: bool
    system_id: int
    component_id: int
    last_source_time_boot_ms: int
    last_receipt_monotonic_ns: int
    telemetry_age_s: float
    freshness_threshold_s: float
    detail: str


TelemetrySample = HeartbeatTelemetry | LocalPositionNedTelemetry | GlobalPositionTelemetry | AttitudeNedFrdTelemetry | LandedStateTelemetry


LANDED_STATES: dict[int, tuple[str, bool | None]] = {
    0: ("UNDEFINED", None),
    1: ("ON_GROUND", True),
    2: ("IN_AIR", False),
    3: ("TAKEOFF", False),
    4: ("LANDING", False),
}


def parse_attitude(
    fields: Mapping[str, Any], *, session_id: str, sequence: int,
    source_sequence: int, system_id: int, component_id: int,
    receipt_monotonic_ns: int,
) -> AttitudeNedFrdTelemetry:
    return AttitudeNedFrdTelemetry(
        session_id, sequence, source_sequence, system_id, component_id,
        _non_negative_int(fields["time_boot_ms"], "time_boot_ms"),
        receipt_monotonic_ns, _finite(fields["roll"], "roll"),
        _finite(fields["pitch"], "pitch"), _finite(fields["yaw"], "yaw"),
    )


def parse_extended_system_state(
    fields: Mapping[str, Any], *, session_id: str, sequence: int,
    source_sequence: int, system_id: int, component_id: int,
    receipt_monotonic_ns: int,
) -> LandedStateTelemetry:
    raw = _non_negative_int(fields["landed_state"], "landed_state")
    name, landed = LANDED_STATES.get(raw, (f"UNKNOWN({raw})", None))
    return LandedStateTelemetry(
        session_id, sequence, source_sequence, system_id, component_id,
        receipt_monotonic_ns, landed, name,
    )


def parse_heartbeat(
    fields: Mapping[str, Any],
    *,
    session_id: str,
    sequence: int,
    source_sequence: int,
    system_id: int,
    component_id: int,
    receipt_monotonic_ns: int,
) -> HeartbeatTelemetry:
    """Parse HEARTBEAT flags and Copter custom mode without pymavlink."""
    base_mode = _non_negative_int(fields["base_mode"], "base_mode")
    custom_mode = _non_negative_int(fields["custom_mode"], "custom_mode")
    if base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        flight_mode = COPTER_MODES.get(custom_mode, f"UNKNOWN({custom_mode})")
    else:
        flight_mode = "CUSTOM_MODE_DISABLED"
    return HeartbeatTelemetry(
        session_id=session_id,
        sequence=sequence,
        source_sequence=source_sequence,
        system_id=system_id,
        component_id=component_id,
        receipt_monotonic_ns=receipt_monotonic_ns,
        armed=bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
        flight_mode=flight_mode,
    )


def parse_local_position_ned(
    fields: Mapping[str, Any],
    *,
    session_id: str,
    sequence: int,
    source_sequence: int,
    system_id: int,
    component_id: int,
    receipt_monotonic_ns: int,
) -> LocalPositionNedTelemetry:
    return LocalPositionNedTelemetry(
        session_id=session_id,
        sequence=sequence,
        source_sequence=source_sequence,
        system_id=system_id,
        component_id=component_id,
        source_time_boot_ms=_non_negative_int(fields["time_boot_ms"], "time_boot_ms"),
        receipt_monotonic_ns=receipt_monotonic_ns,
        north_m=_finite(fields["x"], "x"),
        east_m=_finite(fields["y"], "y"),
        down_m=_finite(fields["z"], "z"),
        velocity_north_m_s=_finite(fields["vx"], "vx"),
        velocity_east_m_s=_finite(fields["vy"], "vy"),
        velocity_down_m_s=_finite(fields["vz"], "vz"),
    )


def parse_global_position_int(
    fields: Mapping[str, Any],
    *,
    session_id: str,
    sequence: int,
    source_sequence: int,
    system_id: int,
    component_id: int,
    receipt_monotonic_ns: int,
) -> GlobalPositionTelemetry:
    heading_raw = _non_negative_int(fields["hdg"], "hdg")
    return GlobalPositionTelemetry(
        session_id=session_id,
        sequence=sequence,
        source_sequence=source_sequence,
        system_id=system_id,
        component_id=component_id,
        source_time_boot_ms=_non_negative_int(fields["time_boot_ms"], "time_boot_ms"),
        receipt_monotonic_ns=receipt_monotonic_ns,
        latitude_deg=_finite(fields["lat"], "lat") / 10_000_000.0,
        longitude_deg=_finite(fields["lon"], "lon") / 10_000_000.0,
        altitude_m_msl=_finite(fields["alt"], "alt") / 1_000.0,
        relative_altitude_m=_finite(fields["relative_alt"], "relative_alt") / 1_000.0,
        velocity_north_m_s=_finite(fields["vx"], "vx") / 100.0,
        velocity_east_m_s=_finite(fields["vy"], "vy") / 100.0,
        velocity_down_m_s=_finite(fields["vz"], "vz") / 100.0,
        heading_deg=None if heading_raw == 65_535 else heading_raw / 100.0,
    )


def serialize_telemetry(value: TelemetrySample | TelemetryStatus) -> str:
    """Return a stable representation for tests, reports and observer output."""
    return json.dumps(asdict(value), indent=2, sort_keys=True, ensure_ascii=True) + "\n"


class TelemetryCore:
    """Accept ordered MAVLink telemetry and maintain explicit health state."""

    def __init__(self, session_prefix: str, freshness_threshold_s: float) -> None:
        if not session_prefix:
            raise ValueError("session_prefix must not be empty")
        if not isfinite(freshness_threshold_s) or freshness_threshold_s <= 0:
            raise ValueError("freshness_threshold_s must be finite and positive")
        self.session_prefix = session_prefix
        self.freshness_threshold_s = freshness_threshold_s
        self.session_id = ""
        self.sequence = 0
        self.health = TelemetryHealth.DISCONNECTED
        self.system_id = 0
        self.component_id = 0
        self._session_number = 0
        self._last_source_sequence: int | None = None
        self._last_receipt_ns = 0
        self._last_position_receipt_ns = 0
        self._last_position_source_ms = 0
        self._last_local_source_ms: int | None = None
        self._last_global_source_ms: int | None = None
        self._last_attitude_source_ms: int | None = None
        self._detail = "waiting for MAVLink heartbeat"

    def _new_session(self, system_id: int, component_id: int) -> None:
        self._session_number += 1
        self.session_id = f"{self.session_prefix}-{self._session_number:04d}"
        self.system_id = system_id
        self.component_id = component_id
        self._last_source_sequence = None
        self._last_receipt_ns = 0
        self._last_position_receipt_ns = 0
        self._last_position_source_ms = 0
        self._last_local_source_ms = None
        self._last_global_source_ms = None
        self._last_attitude_source_ms = None
        self.health = TelemetryHealth.DEGRADED
        self._detail = "heartbeat received; waiting for local position"

    def _accept_packet(self, source_sequence: int, receipt_ns: int) -> bool:
        source_sequence = _non_negative_int(source_sequence, "source_sequence")
        receipt_ns = _non_negative_int(receipt_ns, "receipt_monotonic_ns")
        if source_sequence > 255:
            raise ValueError("source_sequence must be an 8-bit MAVLink sequence")
        if receipt_ns < self._last_receipt_ns:
            return False
        if self._last_source_sequence is not None:
            delta = (source_sequence - self._last_source_sequence) % 256
            if delta == 0 or delta > 127:
                return False
        self._last_source_sequence = source_sequence
        self._last_receipt_ns = receipt_ns
        self.sequence += 1
        return True

    def ingest_heartbeat(
        self,
        fields: Mapping[str, Any],
        *,
        source_sequence: int,
        system_id: int,
        component_id: int,
        receipt_monotonic_ns: int,
    ) -> HeartbeatTelemetry | None:
        if self.health is TelemetryHealth.DISCONNECTED or (
            self.system_id != system_id or self.component_id != component_id
        ):
            self._new_session(system_id, component_id)
        if not self._accept_packet(source_sequence, receipt_monotonic_ns):
            return None
        return parse_heartbeat(
            fields,
            session_id=self.session_id,
            sequence=self.sequence,
            source_sequence=source_sequence,
            system_id=system_id,
            component_id=component_id,
            receipt_monotonic_ns=receipt_monotonic_ns,
        )

    def ingest_local_position(
        self,
        fields: Mapping[str, Any],
        *,
        source_sequence: int,
        system_id: int,
        component_id: int,
        receipt_monotonic_ns: int,
    ) -> LocalPositionNedTelemetry | None:
        source_ms = _non_negative_int(fields["time_boot_ms"], "time_boot_ms")
        if not self._can_accept_position(
            source_ms, self._last_local_source_ms, system_id, component_id
        ):
            return None
        if not self._accept_packet(source_sequence, receipt_monotonic_ns):
            return None
        sample = parse_local_position_ned(
            fields,
            session_id=self.session_id,
            sequence=self.sequence,
            source_sequence=source_sequence,
            system_id=system_id,
            component_id=component_id,
            receipt_monotonic_ns=receipt_monotonic_ns,
        )
        self._last_local_source_ms = source_ms
        self._position_received(source_ms, receipt_monotonic_ns)
        return sample

    def ingest_global_position(
        self,
        fields: Mapping[str, Any],
        *,
        source_sequence: int,
        system_id: int,
        component_id: int,
        receipt_monotonic_ns: int,
    ) -> GlobalPositionTelemetry | None:
        source_ms = _non_negative_int(fields["time_boot_ms"], "time_boot_ms")
        if not self._can_accept_position(
            source_ms, self._last_global_source_ms, system_id, component_id
        ):
            return None
        if not self._accept_packet(source_sequence, receipt_monotonic_ns):
            return None
        sample = parse_global_position_int(
            fields,
            session_id=self.session_id,
            sequence=self.sequence,
            source_sequence=source_sequence,
            system_id=system_id,
            component_id=component_id,
            receipt_monotonic_ns=receipt_monotonic_ns,
        )
        self._last_global_source_ms = source_ms
        self._position_received(source_ms, receipt_monotonic_ns)
        return sample

    def ingest_attitude(
        self, fields: Mapping[str, Any], *, source_sequence: int,
        system_id: int, component_id: int, receipt_monotonic_ns: int,
    ) -> AttitudeNedFrdTelemetry | None:
        source_ms = _non_negative_int(fields["time_boot_ms"], "time_boot_ms")
        if not self._can_accept_position(source_ms, self._last_attitude_source_ms, system_id, component_id):
            return None
        if not self._accept_packet(source_sequence, receipt_monotonic_ns):
            return None
        sample = parse_attitude(
            fields, session_id=self.session_id, sequence=self.sequence,
            source_sequence=source_sequence, system_id=system_id,
            component_id=component_id, receipt_monotonic_ns=receipt_monotonic_ns,
        )
        self._last_attitude_source_ms = source_ms
        return sample

    def ingest_extended_system_state(
        self, fields: Mapping[str, Any], *, source_sequence: int,
        system_id: int, component_id: int, receipt_monotonic_ns: int,
    ) -> LandedStateTelemetry | None:
        if self.health is TelemetryHealth.DISCONNECTED or system_id != self.system_id or component_id != self.component_id:
            return None
        if not self._accept_packet(source_sequence, receipt_monotonic_ns):
            return None
        return parse_extended_system_state(
            fields, session_id=self.session_id, sequence=self.sequence,
            source_sequence=source_sequence, system_id=system_id,
            component_id=component_id, receipt_monotonic_ns=receipt_monotonic_ns,
        )

    def _can_accept_position(
        self,
        source_ms: int,
        previous_ms: int | None,
        system_id: int,
        component_id: int,
    ) -> bool:
        return (
            self.health is not TelemetryHealth.DISCONNECTED
            and system_id == self.system_id
            and component_id == self.component_id
            and (previous_ms is None or source_ms > previous_ms)
        )

    def _position_received(self, source_ms: int, receipt_ns: int) -> None:
        self._last_position_source_ms = max(self._last_position_source_ms, source_ms)
        self._last_position_receipt_ns = receipt_ns
        self.health = TelemetryHealth.CONNECTED
        self._detail = "heartbeat and advancing position telemetry received"

    def disconnect(self, receipt_monotonic_ns: int, detail: str) -> None:
        _non_negative_int(receipt_monotonic_ns, "receipt_monotonic_ns")
        if self.health is not TelemetryHealth.DISCONNECTED:
            self.sequence += 1
        self.health = TelemetryHealth.DISCONNECTED
        self._detail = detail or "MAVLink connection lost"
        self._last_source_sequence = None

    def status(self, now_monotonic_ns: int) -> TelemetryStatus:
        now_ns = _non_negative_int(now_monotonic_ns, "now_monotonic_ns")
        if self._last_position_receipt_ns:
            age_s = max(0.0, (now_ns - self._last_position_receipt_ns) / 1_000_000_000.0)
        else:
            age_s = -1.0
        if (
            self.health in (TelemetryHealth.CONNECTED, TelemetryHealth.STALE)
            and age_s > self.freshness_threshold_s
        ):
            if self.health is not TelemetryHealth.STALE:
                self.sequence += 1
            self.health = TelemetryHealth.STALE
            self._detail = "position telemetry exceeded freshness threshold"
        return TelemetryStatus(
            session_id=self.session_id,
            sequence=self.sequence,
            health=self.health,
            connected=self.health is not TelemetryHealth.DISCONNECTED,
            system_id=self.system_id,
            component_id=self.component_id,
            last_source_time_boot_ms=self._last_position_source_ms,
            last_receipt_monotonic_ns=self._last_position_receipt_ns,
            telemetry_age_s=age_s,
            freshness_threshold_s=self.freshness_threshold_s,
            detail=self._detail,
        )
