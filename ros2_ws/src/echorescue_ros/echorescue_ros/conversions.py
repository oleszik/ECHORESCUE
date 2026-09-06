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
from echorescue_interfaces.action import MoveGrid
from echorescue_interfaces.msg import (
    AgentState,
    BridgeStatus,
    CellObservation as CellObservationMsg,
    SensorObservation as SensorObservationMsg,
    SurvivorEvidence as SurvivorEvidenceMsg,
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
