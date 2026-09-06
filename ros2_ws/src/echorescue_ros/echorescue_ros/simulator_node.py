from time import monotonic_ns

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from echorescue.bridge_contracts import BridgeHeartbeat, CommandStatus, MovementResult
from echorescue.closed_loop import ClosedLoopSimulatorBackend
from echorescue.config import SimulationConfig
from echorescue_interfaces.action import MoveGrid
from echorescue_interfaces.msg import AgentState, BridgeStatus, SensorObservation
from echorescue_ros.conversions import (
    command_from_goal,
    heartbeat_from_msg,
    heartbeat_to_msg,
    observation_to_msg,
    result_to_msg,
    state_to_msg,
)


DATA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=32,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class SimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("echorescue_simulator")
        defaults = {
            "width": 11,
            "height": 9,
            "seed": 7,
            "obstacle_density": 0.04,
            "survivor_count": 2,
            "sensor_range": 4,
            "battery_capacity": 300.0,
            "command_lease_timeout_s": 2.0,
            "drop_observations_after": -1,
            "drop_states_after": -1,
            "duplicate_messages": False,
            "publish_delayed_messages": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config = SimulationConfig(
            width=self._int("width"),
            height=self._int("height"),
            seed=self._int("seed"),
            obstacle_density=self._float("obstacle_density"),
            survivor_count=self._int("survivor_count"),
            sensor_range=self._int("sensor_range"),
            battery_capacity=self._float("battery_capacity"),
            drone_count=1,
        )
        self.backend = ClosedLoopSimulatorBackend(
            config,
            command_lease_timeout_s=self._float("command_lease_timeout_s"),
            clock_ns=monotonic_ns,
        )
        self._observation_pub = self.create_publisher(SensorObservation, "/echorescue/observations", DATA_QOS)
        self._state_pub = self.create_publisher(AgentState, "/echorescue/agent_state", DATA_QOS)
        self._status_pub = self.create_publisher(BridgeStatus, "/echorescue/simulator/status", 10)
        self._mission_status_sub = self.create_subscription(BridgeStatus, "/echorescue/mission/status", self._mission_status, 10)
        self._action_server = ActionServer(
            self,
            MoveGrid,
            "/echorescue/move_grid",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._previous_observation = None
        self._previous_state = None
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info("simulator backend ready; waiting for a mission session")

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _mission_status(self, message: BridgeStatus) -> None:
        previous = self.backend.active_session
        try:
            accepted = self.backend.receive_mission_heartbeat(heartbeat_from_msg(message))
        except ValueError as error:
            self.get_logger().warning(f"invalid mission heartbeat: {error}")
            return
        if not accepted:
            self.get_logger().warning(f"rejected session takeover from {message.session_id}")
            return
        if previous != self.backend.active_session:
            self.get_logger().info(f"session synchronized: {self.backend.active_session}")
            self._publish_snapshot()
        elif self.backend.state_sequence == 0:
            # Volatile QoS intentionally avoids stale data across restarts.  A
            # newly discovered subscriber may have missed the first snapshot,
            # so repeat the cached sequence-zero pair until movement begins.
            self._republish_snapshot()

    def _goal(self, goal: MoveGrid.Goal) -> GoalResponse:
        try:
            command_from_goal(goal)
        except ValueError as error:
            self.get_logger().warning(f"rejected invalid movement goal: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle: object) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: object) -> MoveGrid.Result:
        command = command_from_goal(goal_handle.request)
        feedback = MoveGrid.Feedback()
        feedback.command_id = command.command_id
        feedback.status = feedback.FEEDBACK_ACCEPTED
        feedback.state_sequence = self.backend.state_sequence
        feedback.actual_x = self.backend.position.x
        feedback.actual_y = self.backend.position.y
        goal_handle.publish_feedback(feedback)
        if goal_handle.is_cancel_requested:
            result = MovementResult(
                command.session_id,
                command.agent_id,
                command.command_id,
                CommandStatus.CANCELED,
                self.backend.state_sequence,
                self.backend.sim_time,
                self.backend.position,
                "canceled_before_atomic_step",
            )
            goal_handle.canceled()
            return result_to_msg(result)
        result = self.backend.execute(command)
        if result.status is CommandStatus.COMPLETED:
            goal_handle.succeed()
            self.get_logger().info(f"completed {command.command_id}: {command.source} -> {command.target}")
            self._publish_snapshot()
        elif result.status is CommandStatus.CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
            self.get_logger().warning(f"{result.status.value} {command.command_id}: {result.detail}")
        return result_to_msg(result)

    def _publish_snapshot(self) -> None:
        observation = observation_to_msg(self.backend.observation())
        state = state_to_msg(self.backend.state_report())
        sequence = self.backend.state_sequence
        drop_observation = self._int("drop_observations_after") >= 0 and sequence >= self._int("drop_observations_after")
        drop_state = self._int("drop_states_after") >= 0 and sequence >= self._int("drop_states_after")
        delayed = bool(self.get_parameter("publish_delayed_messages").value)
        duplicate = bool(self.get_parameter("duplicate_messages").value)
        if delayed and self._previous_observation is not None:
            self._observation_pub.publish(self._previous_observation)
        if delayed and self._previous_state is not None:
            self._state_pub.publish(self._previous_state)
        if not drop_observation:
            self._observation_pub.publish(observation)
            if duplicate:
                self._observation_pub.publish(observation)
        if not drop_state:
            self._state_pub.publish(state)
            if duplicate:
                self._state_pub.publish(state)
        self._previous_observation = observation
        self._previous_state = state

    def _republish_snapshot(self) -> None:
        if self._previous_observation is not None:
            self._observation_pub.publish(self._previous_observation)
        if self._previous_state is not None:
            self._state_pub.publish(self._previous_state)

    def _publish_status(self) -> None:
        was_stopped = self.backend.stopped
        heartbeat = self.backend.heartbeat()
        self._status_pub.publish(heartbeat_to_msg(heartbeat))
        if not was_stopped and self.backend.stopped:
            self.get_logger().error("controlled backend stop: mission heartbeat timeout")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
