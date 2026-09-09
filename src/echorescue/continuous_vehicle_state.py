"""Continuous 3D vehicle-state contracts at the ArduPilot/EchoRescue boundary.

ArduPilot local telemetry uses a north-east-down (NED) world frame and a
forward-right-down (FRD) body frame.  EchoRescue exposes east-north-up (ENU)
world coordinates and a forward-left-up (FLU) body convention.  This module is
ROS-independent and never reads simulator Ground Truth.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import asin, atan2, cos, isfinite, pi, sin

from echorescue.mavlink_telemetry import (
    AttitudeNedFrdTelemetry,
    GlobalPositionTelemetry,
    HeartbeatTelemetry,
    LandedStateTelemetry,
    LocalPositionNedTelemetry,
    TelemetryHealth,
    TelemetryStatus,
)


LOCAL_NED_FRAME = "mavlink/local_ned"
BODY_FRD_FRAME = "mavlink/body_frd"
WORLD_ENU_FRAME = "echorescue/world_enu"
BODY_FLU_FRAME = "echorescue/body_flu"
ANGLE_CONVENTION = "right_handed_radians_ccw_from_east"
WORLD_ORIGIN_POLICY = "ardupilot_local_ned_at_sitl_startup"


def normalize_angle_rad(value: float) -> float:
    """Normalize a finite angle to the half-open range [-pi, pi)."""
    if not isfinite(value):
        raise ValueError("angle must be finite")
    normalized = (value + pi) % (2.0 * pi) - pi
    return 0.0 if normalized == 0.0 else normalized


def normalize_angle_deg(value: float) -> float:
    """Normalize a finite angle to the half-open range [-180, 180)."""
    if not isfinite(value):
        raise ValueError("angle must be finite")
    normalized = (value + 180.0) % 360.0 - 180.0
    return 0.0 if normalized == 0.0 else normalized


def ned_to_enu(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map a world vector from NED (north, east, down) to ENU."""
    north, east, down = vector
    if not all(isfinite(value) for value in vector):
        raise ValueError("NED vector components must be finite")
    return east, north, -down


def enu_to_ned(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Inverse of :func:`ned_to_enu`."""
    east, north, up = vector
    if not all(isfinite(value) for value in vector):
        raise ValueError("ENU vector components must be finite")
    return north, east, -up


def frd_to_flu(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map a body vector from forward-right-down to forward-left-up."""
    forward, right, down = vector
    if not all(isfinite(value) for value in vector):
        raise ValueError("FRD vector components must be finite")
    return forward, -right, -down


def flu_to_frd(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Inverse of :func:`frd_to_flu`."""
    forward, left, up = vector
    if not all(isfinite(value) for value in vector):
        raise ValueError("FLU vector components must be finite")
    return forward, -left, -up


def heading_ned_deg_to_enu_deg(heading_deg: float) -> float:
    """Convert clockwise-from-north heading to CCW-from-east ENU yaw."""
    return normalize_angle_deg(90.0 - heading_deg)


def heading_enu_deg_to_ned_deg(yaw_deg: float) -> float:
    """Inverse heading mapping, normalized to [0, 360)."""
    if not isfinite(yaw_deg):
        raise ValueError("yaw must be finite")
    return (90.0 - yaw_deg) % 360.0


def _multiply(left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][inner] * right[inner][column] for inner in range(3)) for column in range(3))
        for row in range(3)
    )


def _transpose(matrix: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _rotation_zyx(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _euler_zyx(matrix: tuple[tuple[float, ...], ...]) -> tuple[float, float, float]:
    pitch = asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(cos(pitch)) > 1e-12:
        roll = atan2(matrix[2][1], matrix[2][2])
        yaw = atan2(matrix[1][0], matrix[0][0])
    else:
        roll = 0.0
        yaw = atan2(-matrix[0][1], matrix[1][1])
    return tuple(normalize_angle_rad(value) for value in (roll, pitch, yaw))  # type: ignore[return-value]


_ENU_FROM_NED = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_NED_FROM_ENU = _transpose(_ENU_FROM_NED)
_FLU_FROM_FRD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
_FRD_FROM_FLU = _transpose(_FLU_FROM_FRD)


def _change_orientation_basis(
    rotation: tuple[tuple[float, ...], ...],
    *,
    output_world_from_input_world: tuple[tuple[float, ...], ...],
    input_body_from_output_body: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    """Change world and body bases for a body-to-world rotation matrix."""
    return _multiply(
        _multiply(output_world_from_input_world, rotation),
        input_body_from_output_body,
    )


def attitude_ned_frd_to_enu_flu(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """Convert ZYX attitude from NED/FRD to ENU/FLU."""
    if not all(isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("attitude angles must be finite")
    converted = _change_orientation_basis(
        _rotation_zyx(roll, pitch, yaw),
        output_world_from_input_world=_ENU_FROM_NED,
        input_body_from_output_body=_FRD_FROM_FLU,
    )
    return _euler_zyx(converted)


def attitude_enu_flu_to_ned_frd(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """Inverse ZYX attitude conversion for round-trip validation."""
    if not all(isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("attitude angles must be finite")
    converted = _change_orientation_basis(
        _rotation_zyx(roll, pitch, yaw),
        output_world_from_input_world=_NED_FROM_ENU,
        input_body_from_output_body=_FLU_FROM_FRD,
    )
    return _euler_zyx(converted)


@dataclass(frozen=True, slots=True)
class ContinuousVehicleState:
    vehicle_id: str
    session_id: str
    sequence: int
    source_sequence: int
    system_id: int
    component_id: int
    source_time_boot_ms: int
    receipt_monotonic_ns: int
    source_frame: str
    output_frame: str
    source_body_frame: str
    output_body_frame: str
    world_origin_policy: str
    angle_convention: str
    x_m: float
    y_m: float
    z_m: float
    vx_m_s: float
    vy_m_s: float
    vz_m_s: float
    position_valid: bool
    velocity_valid: bool
    has_attitude: bool
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    attitude_source_time_boot_ms: int
    has_heading: bool
    heading_deg: float
    heading_source_time_boot_ms: int
    armed: bool
    has_landed_state: bool
    landed: bool
    telemetry_health: TelemetryHealth


class ContinuousStateAssembler:
    """Join ordered receive-only samples without crossing MAVLink sessions."""

    def __init__(
        self,
        vehicle_id: str,
        *,
        source_frame: str = LOCAL_NED_FRAME,
        output_frame: str = WORLD_ENU_FRAME,
        source_body_frame: str = BODY_FRD_FRAME,
        output_body_frame: str = BODY_FLU_FRAME,
        world_origin_policy: str = WORLD_ORIGIN_POLICY,
        angle_convention: str = ANGLE_CONVENTION,
    ) -> None:
        values = (vehicle_id, source_frame, output_frame, source_body_frame, output_body_frame, world_origin_policy, angle_convention)
        if not all(values):
            raise ValueError("vehicle and frame contract values must not be empty")
        if angle_convention != ANGLE_CONVENTION:
            raise ValueError(f"unsupported angle convention: {angle_convention}")
        if world_origin_policy != WORLD_ORIGIN_POLICY:
            raise ValueError(f"unsupported world origin policy: {world_origin_policy}")
        self.vehicle_id = vehicle_id
        self.source_frame = source_frame
        self.output_frame = output_frame
        self.source_body_frame = source_body_frame
        self.output_body_frame = output_body_frame
        self.world_origin_policy = world_origin_policy
        self.angle_convention = angle_convention
        self._session_id = ""
        self._last_source_time_boot_ms: int | None = None
        self._heartbeat: HeartbeatTelemetry | None = None
        self._attitude: AttitudeNedFrdTelemetry | None = None
        self._heading: GlobalPositionTelemetry | None = None
        self._landed: LandedStateTelemetry | None = None

    def _same_vehicle(self, sample: object) -> bool:
        return (
            getattr(sample, "session_id", None) == self._session_id
            and getattr(sample, "system_id", None) == getattr(self._heartbeat, "system_id", None)
            and getattr(sample, "component_id", None) == getattr(self._heartbeat, "component_id", None)
        )

    def ingest_heartbeat(self, sample: HeartbeatTelemetry) -> None:
        if sample.session_id != self._session_id:
            self._session_id = sample.session_id
            self._last_source_time_boot_ms = None
            self._attitude = None
            self._heading = None
            self._landed = None
        self._heartbeat = sample

    def ingest_attitude(self, sample: AttitudeNedFrdTelemetry) -> None:
        if self._same_vehicle(sample) and (self._attitude is None or sample.source_time_boot_ms > self._attitude.source_time_boot_ms):
            self._attitude = sample

    def ingest_heading(self, sample: GlobalPositionTelemetry) -> None:
        if self._same_vehicle(sample) and sample.heading_deg is not None and (self._heading is None or sample.source_time_boot_ms > self._heading.source_time_boot_ms):
            self._heading = sample

    def ingest_landed_state(self, sample: LandedStateTelemetry) -> None:
        if self._same_vehicle(sample):
            self._landed = sample

    def convert(self, sample: LocalPositionNedTelemetry, status: TelemetryStatus) -> ContinuousVehicleState | None:
        if (
            not self._same_vehicle(sample)
            or status.session_id != sample.session_id
            or sample.frame_id != self.source_frame
        ):
            return None
        if self._last_source_time_boot_ms is not None and sample.source_time_boot_ms <= self._last_source_time_boot_ms:
            return None
        self._last_source_time_boot_ms = sample.source_time_boot_ms
        position = ned_to_enu((sample.north_m, sample.east_m, sample.down_m))
        velocity = ned_to_enu((sample.velocity_north_m_s, sample.velocity_east_m_s, sample.velocity_down_m_s))
        has_attitude = self._attitude is not None and self._attitude.source_time_boot_ms <= sample.source_time_boot_ms
        attitude = attitude_ned_frd_to_enu_flu(self._attitude.roll_rad, self._attitude.pitch_rad, self._attitude.yaw_rad) if has_attitude and self._attitude else (0.0, 0.0, 0.0)
        has_heading = (
            self._heading is not None
            and self._heading.heading_deg is not None
            and self._heading.source_time_boot_ms <= sample.source_time_boot_ms
        )
        heading = heading_ned_deg_to_enu_deg(self._heading.heading_deg) if has_heading and self._heading is not None and self._heading.heading_deg is not None else 0.0
        return ContinuousVehicleState(
            vehicle_id=self.vehicle_id,
            session_id=sample.session_id,
            sequence=sample.sequence,
            source_sequence=sample.source_sequence,
            system_id=sample.system_id,
            component_id=sample.component_id,
            source_time_boot_ms=sample.source_time_boot_ms,
            receipt_monotonic_ns=sample.receipt_monotonic_ns,
            source_frame=self.source_frame,
            output_frame=self.output_frame,
            source_body_frame=self.source_body_frame,
            output_body_frame=self.output_body_frame,
            world_origin_policy=self.world_origin_policy,
            angle_convention=self.angle_convention,
            x_m=position[0], y_m=position[1], z_m=position[2],
            vx_m_s=velocity[0], vy_m_s=velocity[1], vz_m_s=velocity[2],
            position_valid=True, velocity_valid=True,
            has_attitude=has_attitude,
            roll_rad=attitude[0], pitch_rad=attitude[1], yaw_rad=attitude[2],
            attitude_source_time_boot_ms=self._attitude.source_time_boot_ms if has_attitude and self._attitude else 0,
            has_heading=has_heading, heading_deg=heading,
            heading_source_time_boot_ms=self._heading.source_time_boot_ms if has_heading and self._heading else 0,
            armed=self._heartbeat.armed if self._heartbeat else False,
            has_landed_state=self._landed is not None and self._landed.landed is not None,
            landed=self._landed.landed if self._landed is not None and self._landed.landed is not None else False,
            telemetry_health=status.health,
        )


def serialize_continuous_state(value: ContinuousVehicleState) -> str:
    return json.dumps(asdict(value), indent=2, sort_keys=True, ensure_ascii=True) + "\n"
