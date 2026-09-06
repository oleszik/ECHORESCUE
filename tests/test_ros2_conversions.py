import pytest

pytest.importorskip("rclpy", reason="ROS 2 conversion tests run in the separate ROS job")

from echorescue.bridge_contracts import (  # noqa: E402
    AgentStateReport,
    BridgeHealth,
    BridgeHeartbeat,
    CellObservation,
    CommandStatus,
    GRID_FRAME,
    MovementCommand,
    MovementResult,
    SensorObservation,
    SurvivorEvidence,
)
from echorescue.models import CellState, Position  # noqa: E402
from echorescue_ros.conversions import (  # noqa: E402
    command_from_goal,
    command_to_goal,
    heartbeat_from_msg,
    heartbeat_to_msg,
    observation_from_msg,
    observation_to_msg,
    result_from_msg,
    result_to_msg,
    state_from_msg,
    state_to_msg,
)


def test_observation_round_trip() -> None:
    source = SensorObservation(
        "session-one",
        "drone-1",
        "obs-4",
        4,
        4.25,
        GRID_FRAME,
        Position(2, 3),
        (
            CellObservation(Position(2, 3), CellState.FREE, 1.0),
            CellObservation(Position(3, 3), CellState.OCCUPIED, 0.75),
        ),
        (SurvivorEvidence(Position(4, 3), 0.875),),
    )
    assert observation_from_msg(observation_to_msg(source)) == source


def test_state_and_heartbeat_round_trip() -> None:
    state = AgentStateReport(
        "session-one", "drone-1", 5, 5.0, GRID_FRAME, Position(3, 2), 91.25, "cmd-5"
    )
    heartbeat = BridgeHeartbeat(
        "session-one", "simulator", 8, 5.0, BridgeHealth.READY
    )
    assert state_from_msg(state_to_msg(state)) == state
    assert heartbeat_from_msg(heartbeat_to_msg(heartbeat)) == heartbeat


def test_action_goal_and_result_round_trip() -> None:
    command = MovementCommand(
        "session-one", "drone-1", "cmd-5", 4, 4.0, 6.0, Position(2, 2), Position(3, 2)
    )
    result = MovementResult(
        "session-one",
        "drone-1",
        "cmd-5",
        CommandStatus.COMPLETED,
        5,
        5.0,
        Position(3, 2),
        "done",
    )
    assert command_from_goal(command_to_goal(command)) == command
    assert result_from_msg(result_to_msg(result)) == result
