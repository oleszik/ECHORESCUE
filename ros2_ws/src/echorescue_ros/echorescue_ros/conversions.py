from builtin_interfaces.msg import Time

from echorescue.bridge_contracts import (
    AgentStateReport,
    BridgeHealth,
    BridgeHeartbeat,
    CellObservation,
    CommandStatus,
    MovementCommand,
    MovementResult,
    SensorObservation,
    SurvivorEvidence,
)
from echorescue.models import CellState, Position
from echorescue.mavlink_telemetry import (
    GlobalPositionTelemetry,
    HeartbeatTelemetry,
    LocalPositionNedTelemetry,
    TelemetryHealth,
    TelemetryStatus,
)
from echorescue.continuous_vehicle_state import ContinuousVehicleState
from echorescue.waypoint_mission import TargetRequest, WaypointMissionEvent
from echorescue_interfaces.action import MoveGrid
from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D,
    AgentState,
    BridgeStatus,
    CellObservation as CellObservationMsg,
    SensorObservation as SensorObservationMsg,
    SurvivorEvidence as SurvivorEvidenceMsg,
    MavlinkGlobalPosition,
    MavlinkLocalPositionNed,
    MavlinkTelemetryStatus,
    MavlinkVehicleState,
    WaypointMissionEvent as WaypointMissionEventMsg,
    WaypointTarget,
)


def waypoint_target_to_msg(
    request: TargetRequest,
    stamp: Time,
    *,
    sequence: int,
    transmit_monotonic_ns: int,
    transmit_source_time_boot_ms: int,
) -> WaypointTarget:
    message = WaypointTarget()
    message.header.stamp = stamp
    message.header.frame_id = "echorescue/world_enu"
    message.session_id = request.session_id
    message.sequence = sequence
    message.target_id = request.target.target_id
    message.east_m = request.target.east_m
    message.north_m = request.target.north_m
    message.up_m = request.target.up_m
    message.has_heading = request.target.heading_enu_deg is not None
    message.heading_enu_deg = request.target.heading_enu_deg or 0.0
    message.coordinate_frame = request.ned.coordinate_frame
    message.type_mask = request.ned.type_mask
    message.ned_north_m = request.ned.north_m
    message.ned_east_m = request.ned.east_m
    message.ned_down_m = request.ned.down_m
    message.ned_yaw_rad = request.ned.yaw_rad
    message.transmit_monotonic_ns = transmit_monotonic_ns
    message.transmit_source_time_boot_ms = transmit_source_time_boot_ms
    return message


def waypoint_event_to_msg(source: WaypointMissionEvent, stamp: Time) -> WaypointMissionEventMsg:
    message = WaypointMissionEventMsg()
    message.header.stamp = stamp
    message.header.frame_id = "echorescue/world_enu"
    for field in ("sequence", "monotonic_ns", "session_id", "phase", "event", "target_id", "detail"):
        setattr(message, field, getattr(source, field))
    return message


def continuous_vehicle_state_to_msg(source: ContinuousVehicleState, stamp: Time) -> EchoRescueVehicleState3D:
    message = EchoRescueVehicleState3D()
    message.header.stamp = stamp
    message.header.frame_id = source.output_frame
    for field in (
        "vehicle_id", "session_id", "sequence", "source_sequence", "system_id",
        "component_id", "source_time_boot_ms", "receipt_monotonic_ns",
        "source_frame", "output_frame", "source_body_frame", "output_body_frame",
        "world_origin_policy", "angle_convention", "x_m", "y_m", "z_m",
        "vx_m_s", "vy_m_s", "vz_m_s", "position_valid", "velocity_valid",
        "has_attitude", "roll_rad", "pitch_rad", "yaw_rad",
        "attitude_source_time_boot_ms", "has_heading", "heading_deg",
        "heading_source_time_boot_ms", "armed", "has_landed_state", "landed",
    ):
        setattr(message, field, getattr(source, field))
    message.telemetry_health = {
        TelemetryHealth.CONNECTED: message.CONNECTED,
        TelemetryHealth.DEGRADED: message.DEGRADED,
        TelemetryHealth.STALE: message.STALE,
        TelemetryHealth.DISCONNECTED: message.DISCONNECTED,
    }[source.telemetry_health]
    return message


def continuous_vehicle_state_from_msg(message: EchoRescueVehicleState3D) -> ContinuousVehicleState:
    health_values = {
        message.CONNECTED: TelemetryHealth.CONNECTED,
        message.DEGRADED: TelemetryHealth.DEGRADED,
        message.STALE: TelemetryHealth.STALE,
        message.DISCONNECTED: TelemetryHealth.DISCONNECTED,
    }
    health_value = int(message.telemetry_health)
    try:
        health = health_values[health_value]
    except KeyError as error:
        raise ValueError(f"unknown telemetry_health value: {health_value}") from error
    return ContinuousVehicleState(
        vehicle_id=message.vehicle_id, session_id=message.session_id,
        sequence=int(message.sequence), source_sequence=int(message.source_sequence),
        system_id=int(message.system_id), component_id=int(message.component_id),
        source_time_boot_ms=int(message.source_time_boot_ms),
        receipt_monotonic_ns=int(message.receipt_monotonic_ns),
        source_frame=message.source_frame, output_frame=message.output_frame,
        source_body_frame=message.source_body_frame, output_body_frame=message.output_body_frame,
        world_origin_policy=message.world_origin_policy, angle_convention=message.angle_convention,
        x_m=float(message.x_m), y_m=float(message.y_m), z_m=float(message.z_m),
        vx_m_s=float(message.vx_m_s), vy_m_s=float(message.vy_m_s), vz_m_s=float(message.vz_m_s),
        position_valid=message.position_valid, velocity_valid=message.velocity_valid,
        has_attitude=message.has_attitude, roll_rad=float(message.roll_rad),
        pitch_rad=float(message.pitch_rad), yaw_rad=float(message.yaw_rad),
        attitude_source_time_boot_ms=int(message.attitude_source_time_boot_ms),
        has_heading=message.has_heading, heading_deg=float(message.heading_deg),
        heading_source_time_boot_ms=int(message.heading_source_time_boot_ms),
        armed=message.armed, has_landed_state=message.has_landed_state,
        landed=message.landed, telemetry_health=health,
    )


def seconds_to_time(value: float) -> Time:
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return Time(sec=seconds, nanosec=nanoseconds)


def time_to_seconds(value: Time) -> float:
    return float(value.sec) + float(value.nanosec) / 1_000_000_000.0


def observation_from_msg(message: SensorObservationMsg) -> SensorObservation:
    states = {message.cells[0].UNKNOWN: CellState.UNKNOWN, message.cells[0].FREE: CellState.FREE, message.cells[0].OCCUPIED: CellState.OCCUPIED} if message.cells else {0: CellState.UNKNOWN, 1: CellState.FREE, 2: CellState.OCCUPIED}
    return SensorObservation(
        message.session_id,
        message.agent_id,
        message.observation_id,
        int(message.sequence),
        time_to_seconds(message.header.stamp),
        message.header.frame_id,
        Position(message.agent_x, message.agent_y),
        tuple(CellObservation(Position(item.x, item.y), states[item.state], float(item.confidence)) for item in message.cells),
        tuple(SurvivorEvidence(Position(item.x, item.y), float(item.confidence), item.detected, item.channel) for item in message.survivors),
    )


def observation_to_msg(observation: SensorObservation) -> SensorObservationMsg:
    message = SensorObservationMsg()
    message.header.stamp = seconds_to_time(observation.sim_time)
    message.header.frame_id = observation.frame_id
    message.session_id = observation.session_id
    message.agent_id = observation.agent_id
    message.observation_id = observation.observation_id
    message.sequence = observation.sequence
    message.agent_x = observation.agent_position.x
    message.agent_y = observation.agent_position.y
    for source in observation.cells:
        item = CellObservationMsg()
        item.x = source.position.x
        item.y = source.position.y
        item.state = {CellState.UNKNOWN: item.UNKNOWN, CellState.FREE: item.FREE, CellState.OCCUPIED: item.OCCUPIED}[source.state]
        item.confidence = source.confidence
        message.cells.append(item)
    for source in observation.survivors:
        item = SurvivorEvidenceMsg()
        item.x = source.position.x
        item.y = source.position.y
        item.confidence = source.confidence
        item.detected = source.detected
        item.channel = source.channel
        message.survivors.append(item)
    return message


def state_from_msg(message: AgentState) -> AgentStateReport:
    return AgentStateReport(message.session_id, message.agent_id, int(message.sequence), time_to_seconds(message.header.stamp), message.header.frame_id, Position(message.x, message.y), float(message.energy_remaining), message.last_command_id, message.motion_state, message.error)


def state_to_msg(report: AgentStateReport) -> AgentState:
    message = AgentState()
    message.header.stamp = seconds_to_time(report.sim_time)
    message.header.frame_id = report.frame_id
    message.session_id = report.session_id
    message.agent_id = report.agent_id
    message.sequence = report.sequence
    message.x = report.position.x
    message.y = report.position.y
    message.energy_remaining = report.energy_remaining
    message.last_command_id = report.last_command_id
    message.motion_state = report.motion_state
    message.error = report.error
    return message


def heartbeat_from_msg(message: BridgeStatus) -> BridgeHeartbeat:
    health = {message.STARTING: BridgeHealth.STARTING, message.READY: BridgeHealth.READY, message.STOPPED: BridgeHealth.STOPPED, message.ERROR: BridgeHealth.ERROR}[message.health]
    return BridgeHeartbeat(message.session_id, message.component_id, int(message.sequence), time_to_seconds(message.header.stamp), health, message.error)


def heartbeat_to_msg(heartbeat: BridgeHeartbeat) -> BridgeStatus:
    message = BridgeStatus()
    message.header.stamp = seconds_to_time(heartbeat.sim_time)
    message.header.frame_id = "echorescue/grid"
    message.session_id = heartbeat.session_id
    message.component_id = heartbeat.component_id
    message.sequence = heartbeat.sequence
    message.health = {BridgeHealth.STARTING: message.STARTING, BridgeHealth.READY: message.READY, BridgeHealth.STOPPED: message.STOPPED, BridgeHealth.ERROR: message.ERROR}[heartbeat.health]
    message.error = heartbeat.error
    return message


def command_to_goal(command: MovementCommand) -> MoveGrid.Goal:
    goal = MoveGrid.Goal()
    goal.session_id = command.session_id
    goal.agent_id = command.agent_id
    goal.command_id = command.command_id
    goal.expected_state_sequence = command.expected_state_sequence
    goal.issued_at = seconds_to_time(command.issued_sim_time)
    goal.valid_until = seconds_to_time(command.valid_until_sim_time)
    goal.source_x, goal.source_y = command.source.x, command.source.y
    goal.target_x, goal.target_y = command.target.x, command.target.y
    return goal


def command_from_goal(goal: MoveGrid.Goal) -> MovementCommand:
    return MovementCommand(goal.session_id, goal.agent_id, goal.command_id, int(goal.expected_state_sequence), time_to_seconds(goal.issued_at), time_to_seconds(goal.valid_until), Position(goal.source_x, goal.source_y), Position(goal.target_x, goal.target_y))


def result_to_msg(result: MovementResult) -> MoveGrid.Result:
    message = MoveGrid.Result()
    message.session_id = result.session_id
    message.agent_id = result.agent_id
    message.command_id = result.command_id
    message.status = {CommandStatus.ACCEPTED: message.ACCEPTED, CommandStatus.COMPLETED: message.COMPLETED, CommandStatus.REJECTED: message.REJECTED, CommandStatus.CANCELED: message.CANCELED, CommandStatus.FAILED: message.FAILED}[result.status]
    message.state_sequence = result.state_sequence
    message.completed_at = seconds_to_time(result.sim_time)
    message.actual_x, message.actual_y = result.actual_position.x, result.actual_position.y
    message.detail = result.detail
    return message


def result_from_msg(message: MoveGrid.Result) -> MovementResult:
    status = {message.ACCEPTED: CommandStatus.ACCEPTED, message.COMPLETED: CommandStatus.COMPLETED, message.REJECTED: CommandStatus.REJECTED, message.CANCELED: CommandStatus.CANCELED, message.FAILED: CommandStatus.FAILED}[message.status]
    return MovementResult(message.session_id, message.agent_id, message.command_id, status, int(message.state_sequence), time_to_seconds(message.completed_at), Position(message.actual_x, message.actual_y), message.detail)


def mavlink_vehicle_state_to_msg(source: HeartbeatTelemetry, stamp: Time) -> MavlinkVehicleState:
    message = MavlinkVehicleState()
    message.header.stamp = stamp
    message.header.frame_id = source.frame_id
    message.session_id = source.session_id
    message.sequence = source.sequence
    message.source_sequence = source.source_sequence
    message.system_id = source.system_id
    message.component_id = source.component_id
    message.receipt_monotonic_ns = source.receipt_monotonic_ns
    message.armed = source.armed
    message.flight_mode = source.flight_mode
    return message


def mavlink_vehicle_state_from_msg(message: MavlinkVehicleState) -> HeartbeatTelemetry:
    return HeartbeatTelemetry(
        session_id=message.session_id,
        sequence=int(message.sequence),
        source_sequence=int(message.source_sequence),
        system_id=int(message.system_id),
        component_id=int(message.component_id),
        receipt_monotonic_ns=int(message.receipt_monotonic_ns),
        armed=message.armed,
        flight_mode=message.flight_mode,
        frame_id=message.header.frame_id,
    )


def mavlink_local_position_to_msg(source: LocalPositionNedTelemetry, stamp: Time) -> MavlinkLocalPositionNed:
    message = MavlinkLocalPositionNed()
    message.header.stamp = stamp
    message.header.frame_id = source.frame_id
    message.session_id = source.session_id
    message.sequence = source.sequence
    message.source_sequence = source.source_sequence
    message.system_id = source.system_id
    message.component_id = source.component_id
    message.source_time_boot_ms = source.source_time_boot_ms
    message.receipt_monotonic_ns = source.receipt_monotonic_ns
    message.north_m = source.north_m
    message.east_m = source.east_m
    message.down_m = source.down_m
    message.velocity_north_m_s = source.velocity_north_m_s
    message.velocity_east_m_s = source.velocity_east_m_s
    message.velocity_down_m_s = source.velocity_down_m_s
    return message


def mavlink_local_position_from_msg(message: MavlinkLocalPositionNed) -> LocalPositionNedTelemetry:
    return LocalPositionNedTelemetry(
        session_id=message.session_id,
        sequence=int(message.sequence),
        source_sequence=int(message.source_sequence),
        system_id=int(message.system_id),
        component_id=int(message.component_id),
        source_time_boot_ms=int(message.source_time_boot_ms),
        receipt_monotonic_ns=int(message.receipt_monotonic_ns),
        north_m=float(message.north_m),
        east_m=float(message.east_m),
        down_m=float(message.down_m),
        velocity_north_m_s=float(message.velocity_north_m_s),
        velocity_east_m_s=float(message.velocity_east_m_s),
        velocity_down_m_s=float(message.velocity_down_m_s),
        frame_id=message.header.frame_id,
    )


def mavlink_global_position_to_msg(source: GlobalPositionTelemetry, stamp: Time) -> MavlinkGlobalPosition:
    message = MavlinkGlobalPosition()
    message.header.stamp = stamp
    message.header.frame_id = source.frame_id
    message.session_id = source.session_id
    message.sequence = source.sequence
    message.source_sequence = source.source_sequence
    message.system_id = source.system_id
    message.component_id = source.component_id
    message.source_time_boot_ms = source.source_time_boot_ms
    message.receipt_monotonic_ns = source.receipt_monotonic_ns
    message.latitude_deg = source.latitude_deg
    message.longitude_deg = source.longitude_deg
    message.altitude_m_msl = source.altitude_m_msl
    message.relative_altitude_m = source.relative_altitude_m
    message.velocity_north_m_s = source.velocity_north_m_s
    message.velocity_east_m_s = source.velocity_east_m_s
    message.velocity_down_m_s = source.velocity_down_m_s
    message.has_heading = source.heading_deg is not None
    message.heading_deg = source.heading_deg if source.heading_deg is not None else 0.0
    return message


def mavlink_global_position_from_msg(message: MavlinkGlobalPosition) -> GlobalPositionTelemetry:
    return GlobalPositionTelemetry(
        session_id=message.session_id,
        sequence=int(message.sequence),
        source_sequence=int(message.source_sequence),
        system_id=int(message.system_id),
        component_id=int(message.component_id),
        source_time_boot_ms=int(message.source_time_boot_ms),
        receipt_monotonic_ns=int(message.receipt_monotonic_ns),
        latitude_deg=float(message.latitude_deg),
        longitude_deg=float(message.longitude_deg),
        altitude_m_msl=float(message.altitude_m_msl),
        relative_altitude_m=float(message.relative_altitude_m),
        velocity_north_m_s=float(message.velocity_north_m_s),
        velocity_east_m_s=float(message.velocity_east_m_s),
        velocity_down_m_s=float(message.velocity_down_m_s),
        heading_deg=float(message.heading_deg) if message.has_heading else None,
        frame_id=message.header.frame_id,
    )


def mavlink_status_to_msg(source: TelemetryStatus, stamp: Time) -> MavlinkTelemetryStatus:
    message = MavlinkTelemetryStatus()
    message.header.stamp = stamp
    message.header.frame_id = "mavlink/bridge"
    message.session_id = source.session_id
    message.sequence = source.sequence
    message.health = {
        TelemetryHealth.CONNECTED: message.CONNECTED,
        TelemetryHealth.DEGRADED: message.DEGRADED,
        TelemetryHealth.STALE: message.STALE,
        TelemetryHealth.DISCONNECTED: message.DISCONNECTED,
    }[source.health]
    message.connected = source.connected
    message.system_id = source.system_id
    message.component_id = source.component_id
    message.last_source_time_boot_ms = source.last_source_time_boot_ms
    message.last_receipt_monotonic_ns = source.last_receipt_monotonic_ns
    message.telemetry_age_s = source.telemetry_age_s
    message.freshness_threshold_s = source.freshness_threshold_s
    message.detail = source.detail
    return message


def mavlink_status_from_msg(message: MavlinkTelemetryStatus) -> TelemetryStatus:
    health = {
        message.CONNECTED: TelemetryHealth.CONNECTED,
        message.DEGRADED: TelemetryHealth.DEGRADED,
        message.STALE: TelemetryHealth.STALE,
        message.DISCONNECTED: TelemetryHealth.DISCONNECTED,
    }[message.health]
    return TelemetryStatus(
        session_id=message.session_id,
        sequence=int(message.sequence),
        health=health,
        connected=message.connected,
        system_id=int(message.system_id),
        component_id=int(message.component_id),
        last_source_time_boot_ms=int(message.last_source_time_boot_ms),
        last_receipt_monotonic_ns=int(message.last_receipt_monotonic_ns),
        telemetry_age_s=float(message.telemetry_age_s),
        freshness_threshold_s=float(message.freshness_threshold_s),
        detail=message.detail,
    )
