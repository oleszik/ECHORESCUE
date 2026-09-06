from pathlib import Path
from time import monotonic_ns
from uuid import uuid4

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from echorescue.bridge_artifacts import build_closed_loop_replay, write_json
from echorescue.bridge_contracts import BridgeHealth, BridgeHeartbeat
from echorescue.closed_loop import ClosedLoopConfig, ClosedLoopMission, MissionBridgeState
from echorescue.models import Position
from echorescue_interfaces.action import MoveGrid
from echorescue_interfaces.msg import AgentState, BridgeStatus, SensorObservation
from echorescue_ros.conversions import (
    command_to_goal,
    heartbeat_from_msg,
    heartbeat_to_msg,
    observation_from_msg,
    result_from_msg,
    state_from_msg,
)


DATA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=32,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("echorescue_mission")
        defaults = {
            "session_id": "",
            "width": 11,
            "height": 9,
            "survivor_count": 2,
            "movement_energy_cost": 1.0,
            "sensor_energy_cost": 0.05,
            "energy_safety_reserve": 20.0,
            "battery_capacity": 300.0,
            "command_timeout_s": 2.0,
            "data_timeout_s": 2.0,
            "report_out": "replays/v0.13-ros2-report.json",
            "replay_out": "replays/v0.13-ros2-replay.json",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        session = str(self.get_parameter("session_id").value) or f"mission-{uuid4()}"
        config = ClosedLoopConfig(
            self._int("width"),
            self._int("height"),
            Position(1, 1),
            self._int("survivor_count"),
            self._float("movement_energy_cost"),
            self._float("sensor_energy_cost"),
            self._float("energy_safety_reserve"),
            command_timeout_s=self._float("command_timeout_s"),
            data_timeout_s=self._float("data_timeout_s"),
            battery_capacity=self._float("battery_capacity"),
        )
        self.mission = ClosedLoopMission(session, "drone-1", config)
        self._status_sequence = 0
        self._goal_in_flight = False
        self._finished = False
        self._status_pub = self.create_publisher(BridgeStatus, "/echorescue/mission/status", 10)
        self._observation_sub = self.create_subscription(SensorObservation, "/echorescue/observations", self._observation, DATA_QOS)
        self._state_sub = self.create_subscription(AgentState, "/echorescue/agent_state", self._state, DATA_QOS)
        self._backend_status_sub = self.create_subscription(BridgeStatus, "/echorescue/simulator/status", self._backend_status, 10)
        self._action_client = ActionClient(self, MoveGrid, "/echorescue/move_grid")
        self.create_timer(0.1, self._heartbeat)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"mission session {session} waiting for synchronized input")

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _observation(self, message: SensorObservation) -> None:
        try:
            self.mission.receive_observation(observation_from_msg(message))
        except (KeyError, ValueError) as error:
            self.get_logger().warning(f"invalid observation ignored: {error}")

    def _state(self, message: AgentState) -> None:
        try:
            self.mission.receive_state(state_from_msg(message))
        except ValueError as error:
            self.get_logger().warning(f"invalid state ignored: {error}")

    def _backend_status(self, message: BridgeStatus) -> None:
        try:
            self.mission.receive_backend_heartbeat(heartbeat_from_msg(message))
        except ValueError as error:
            self.get_logger().warning(f"invalid backend status ignored: {error}")

    def _heartbeat(self) -> None:
        self._status_sequence += 1
        if self.mission.state is MissionBridgeState.COMMUNICATION_ERROR:
            health = BridgeHealth.ERROR
        elif self.mission.state in {MissionBridgeState.CONTROLLED_STOP, MissionBridgeState.COMPLETED}:
            health = BridgeHealth.STOPPED
        else:
            health = BridgeHealth.READY
        sim_time = self.mission.agent_state.sim_time if self.mission.agent_state else 0.0
        heartbeat = BridgeHeartbeat(self.mission.session_id, "mission", self._status_sequence, sim_time, health, self.mission.error)
        self._status_pub.publish(heartbeat_to_msg(heartbeat))

    def _tick(self) -> None:
        if self._finished:
            return
        command = self.mission.tick(monotonic_ns())
        if command is not None and not self._goal_in_flight:
            if not self._action_client.server_is_ready():
                self.get_logger().warning("movement action server unavailable")
            else:
                self._goal_in_flight = True
                self.get_logger().info(f"issuing {command.command_id} -> ({command.target.x}, {command.target.y})")
                future = self._action_client.send_goal_async(command_to_goal(command), feedback_callback=self._feedback)
                future.add_done_callback(self._goal_response)
        if self.mission.state in {MissionBridgeState.COMPLETED, MissionBridgeState.CONTROLLED_STOP}:
            self._finish()

    def _feedback(self, feedback_message: object) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(f"command {feedback.command_id} accepted at state {feedback.state_sequence}")

    def _goal_response(self, future: object) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_in_flight = False
            if self.mission.pending_command is not None:
                self.mission.request_stop("movement_goal_rejected_by_action_server")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future: object) -> None:
        self._goal_in_flight = False
        result = result_from_msg(future.result().result)
        self.mission.receive_result(result)
        self.get_logger().info(f"command {result.command_id} {result.status.value}: {result.detail}")

    def _finish(self) -> None:
        self._finished = True
        report = self.mission.report()
        report_path = write_json(report, Path(str(self.get_parameter("report_out").value)))
        replay_path = write_json(build_closed_loop_replay(self.mission), Path(str(self.get_parameter("replay_out").value)))
        self.get_logger().info(f"closed loop finished: {self.mission.state.value}; report={report_path}; replay={replay_path}")
        self._heartbeat()
        rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
