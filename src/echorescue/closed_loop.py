"""ROS-independent mission controller and simulator backend for v0.13."""

from dataclasses import dataclass
from enum import Enum
from time import monotonic_ns
from typing import Callable

from echorescue.bridge_contracts import (
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
from echorescue.config import SimulationConfig
from echorescue.energy import Battery
from echorescue.environment import GridWorld
from echorescue.knowledge import CellKnowledge, ProbabilisticKnowledgeMap
from echorescue.models import CellState, Position
from echorescue.perception import SurvivorHypothesisTracker
from echorescue.planning import astar, path_to_nearest_frontier
from echorescue.probabilistic import OccupancyEvidence, UNCERTAINTY_PROFILES
from echorescue.sensors import DistanceSensor, uncertain_observations
from echorescue.survivors import SurvivorSensor


class MissionBridgeState(str, Enum):
    WAITING_FOR_INPUT = "waiting_for_input"
    ACTIVE = "active_mission"
    WAITING_FOR_COMMAND = "waiting_for_command_completion"
    COMMUNICATION_ERROR = "communication_or_data_error"
    CONTROLLED_STOP = "controlled_stop"
    COMPLETED = "mission_completed"


@dataclass(frozen=True, slots=True)
class ClosedLoopConfig:
    width: int
    height: int
    base: Position
    expected_survivors: int
    movement_energy_cost: float
    sensor_energy_cost: float
    energy_safety_reserve: float
    command_validity_steps: float = 2.0
    command_timeout_s: float = 2.0
    data_timeout_s: float = 2.0
    battery_capacity: float | None = None

    def __post_init__(self) -> None:
        if self.width < 3 or self.height < 3:
            raise ValueError("closed-loop map must be at least 3x3")
        if not (0 <= self.base.x < self.width and 0 <= self.base.y < self.height):
            raise ValueError("base must be inside the map")
        if self.expected_survivors < 0:
            raise ValueError("expected_survivors must not be negative")
        if self.movement_energy_cost <= 0 or self.sensor_energy_cost < 0:
            raise ValueError("energy costs are invalid")
        if self.energy_safety_reserve < 0:
            raise ValueError("energy reserve must not be negative")
        if min(self.command_validity_steps, self.command_timeout_s, self.data_timeout_s) <= 0:
            raise ValueError("validity and timeout values must be positive")
        if self.battery_capacity is not None and self.battery_capacity <= 0:
            raise ValueError("battery_capacity must be positive when provided")


class ClosedLoopMission:
    """Observation-driven form of the legacy single-agent mission policy.

    This object never accepts or stores a ``GridWorld``.  A command remains
    pending until a command result *and* a matching state report arrive.
    """

    def __init__(self, session_id: str, agent_id: str, config: ClosedLoopConfig) -> None:
        self.session_id = session_id
        self.agent_id = agent_id
        self.config = config
        self.map = ProbabilisticKnowledgeMap(config.width, config.height)
        self.survivors = SurvivorHypothesisTracker(
            minimum_positive_observations=2,
            confirmation_threshold=0.65,
            rejection_threshold=0.12,
            negative_evidence_weight=0.55,
        )
        self.state = MissionBridgeState.WAITING_FOR_INPUT
        self.agent_state: AgentStateReport | None = None
        self.pending_command: MovementCommand | None = None
        self.pending_result: MovementResult | None = None
        self.returning = False
        self.error = ""
        self._latest_observation_sequence = -1
        self._latest_state_sequence = -1
        self._observation_ids: set[str] = set()
        self._command_counter = 0
        self._last_observation_ns: int | None = None
        self._last_state_ns: int | None = None
        self._last_backend_heartbeat_ns: int | None = None
        self._command_sent_ns: int | None = None
        self._events: list[dict[str, object]] = []
        self._frames: list[dict[str, object]] = []
        self._record("mission_created")

    @property
    def events(self) -> tuple[dict[str, object], ...]:
        return tuple(self._events)

    @property
    def frames(self) -> tuple[dict[str, object], ...]:
        return tuple(self._frames)

    @property
    def confirmed_survivors(self) -> frozenset[Position]:
        return frozenset(self.survivors.confirmed_locations)

    def _record(self, event: str, **detail: object) -> None:
        self._events.append({"event": event, "state": self.state.value, **detail})

    def _belongs(self, session_id: str, agent_id: str) -> bool:
        if session_id != self.session_id or agent_id != self.agent_id:
            self._record("foreign_message_ignored", session_id=session_id, agent_id=agent_id)
            return False
        return True

    def receive_backend_heartbeat(
        self, heartbeat: BridgeHeartbeat, received_ns: int | None = None
    ) -> bool:
        if heartbeat.session_id != self.session_id:
            self._record("foreign_heartbeat_ignored", session_id=heartbeat.session_id)
            return False
        now = monotonic_ns() if received_ns is None else received_ns
        self._last_backend_heartbeat_ns = now
        if heartbeat.health is BridgeHealth.ERROR:
            self._enter_error(heartbeat.error or "backend_error")
        else:
            self._try_synchronize()
        return True

    def receive_observation(
        self, observation: SensorObservation, received_ns: int | None = None
    ) -> bool:
        if not self._belongs(observation.session_id, observation.agent_id):
            return False
        if observation.observation_id in self._observation_ids:
            self._record("duplicate_observation_ignored", observation_id=observation.observation_id)
            return False
        if observation.sequence <= self._latest_observation_sequence:
            self._record(
                "stale_observation_ignored",
                observation_id=observation.observation_id,
                sequence=observation.sequence,
            )
            return False
        if self.agent_state is not None and observation.sim_time < self.agent_state.sim_time:
            self._record("late_observation_ignored", observation_id=observation.observation_id)
            return False
        step = int(observation.sim_time)
        self.map.advance(step)
        records = []
        for cell in observation.cells:
            reliability = 0.500001 + 0.499998 * cell.confidence
            occupancy_evidence = OccupancyEvidence(
                observed_step=step,
                source_id=f"{observation.agent_id}:{observation.observation_id}",
                floor=0,
                occupied=cell.state is CellState.OCCUPIED,
                reliability=reliability,
            )
            records.append(
                (
                    cell.position,
                    CellKnowledge(
                        cell.state,
                        step,
                        occupancy_evidence.source_id,
                        (occupancy_evidence,),
                    ),
                )
            )
        self.map.apply(records)
        for survivor_evidence in observation.survivors:
            if survivor_evidence.detected:
                self.survivors.positive(
                    survivor_evidence.position,
                    confidence=survivor_evidence.confidence,
                    channel=survivor_evidence.channel,
                    agent_id=observation.agent_id,
                    step=step,
                )
            else:
                self.survivors.negative(
                    survivor_evidence.position,
                    confidence=survivor_evidence.confidence,
                    channel=survivor_evidence.channel,
                    agent_id=observation.agent_id,
                    step=step,
                )
        self._observation_ids.add(observation.observation_id)
        self._latest_observation_sequence = observation.sequence
        self._last_observation_ns = monotonic_ns() if received_ns is None else received_ns
        self._record("observation_applied", observation_id=observation.observation_id)
        self._try_synchronize()
        return True

    def receive_state(
        self, report: AgentStateReport, received_ns: int | None = None
    ) -> bool:
        if not self._belongs(report.session_id, report.agent_id):
            return False
        if report.sequence <= self._latest_state_sequence:
            self._record("stale_state_ignored", sequence=report.sequence)
            return False
        if self.agent_state is not None and report.sim_time < self.agent_state.sim_time:
            self._record("late_state_ignored", sequence=report.sequence)
            return False
        self.agent_state = report
        self._latest_state_sequence = report.sequence
        self._last_state_ns = monotonic_ns() if received_ns is None else received_ns
        self._record("state_applied", sequence=report.sequence)
        self._try_synchronize()
        return True

    def receive_result(self, result: MovementResult) -> bool:
        if not self._belongs(result.session_id, result.agent_id):
            return False
        if self.pending_command is None or result.command_id != self.pending_command.command_id:
            self._record("unexpected_command_result_ignored", command_id=result.command_id)
            return False
        if self.pending_result is not None:
            self._record("duplicate_command_result_ignored", command_id=result.command_id)
            return False
        self.pending_result = result
        self._record("command_result", command_id=result.command_id, status=result.status.value)
        if result.status is not CommandStatus.COMPLETED:
            self._enter_error(f"command_{result.status.value}:{result.detail}")
        else:
            self._try_synchronize()
        return True

    def _try_synchronize(self) -> None:
        report = self.agent_state
        if (
            report is None
            or self._last_backend_heartbeat_ns is None
            or self._latest_observation_sequence != report.sequence
        ):
            return
        if self.pending_command is not None:
            result = self.pending_result
            if (
                result is None
                or result.status is not CommandStatus.COMPLETED
                or report.last_command_id != self.pending_command.command_id
                or report.position != self.pending_command.target
            ):
                return
            self._record("command_confirmed_by_state", command_id=self.pending_command.command_id)
            self.pending_command = None
            self.pending_result = None
            self._command_sent_ns = None
        if self.state in {MissionBridgeState.WAITING_FOR_INPUT, MissionBridgeState.WAITING_FOR_COMMAND}:
            self.state = MissionBridgeState.ACTIVE
            self._record("input_synchronized", sequence=report.sequence)
            self._capture_frame()

    def _enter_error(self, reason: str) -> None:
        if self.state in {MissionBridgeState.CONTROLLED_STOP, MissionBridgeState.COMPLETED}:
            return
        self.error = reason
        self.state = MissionBridgeState.COMMUNICATION_ERROR
        self._record("communication_error", reason=reason)

    def request_stop(self, reason: str) -> None:
        """Enter the fail-closed path from a transport adapter."""

        self._enter_error(reason)

    def tick(self, now_ns: int | None = None) -> MovementCommand | None:
        now = monotonic_ns() if now_ns is None else now_ns
        if self.state is MissionBridgeState.COMMUNICATION_ERROR:
            self.state = MissionBridgeState.CONTROLLED_STOP
            self._record("controlled_stop", reason=self.error)
            return None
        if self.state in {MissionBridgeState.CONTROLLED_STOP, MissionBridgeState.COMPLETED}:
            return None
        timeout_ns = int(self.config.data_timeout_s * 1_000_000_000)
        timestamps = (
            self._last_observation_ns,
            self._last_state_ns,
            self._last_backend_heartbeat_ns,
        )
        if any(stamp is not None and now - stamp > timeout_ns for stamp in timestamps):
            self._enter_error("stale_input_or_backend_heartbeat")
            return None
        if self.state is MissionBridgeState.WAITING_FOR_COMMAND:
            assert self._command_sent_ns is not None
            if now - self._command_sent_ns > int(self.config.command_timeout_s * 1_000_000_000):
                self._enter_error("command_timeout")
            return None
        if self.state is not MissionBridgeState.ACTIVE or self.agent_state is None:
            return None
        command = self._plan_command()
        if command is not None:
            self.pending_command = command
            self.pending_result = None
            self._command_sent_ns = now
            self.state = MissionBridgeState.WAITING_FOR_COMMAND
            self._record("command_issued", command_id=command.command_id, target=[command.target.x, command.target.y])
        return command

    def _plan_command(self) -> MovementCommand | None:
        assert self.agent_state is not None
        current = self.agent_state.position
        return_path = astar(current, self.config.base, self.map.is_known_free)
        if return_path is None:
            self._enter_error("known_return_path_unavailable")
            return None
        movement_cycle = self.config.movement_energy_cost + self.config.sensor_energy_cost
        required = max(0, len(return_path) - 1) * movement_cycle + self.config.energy_safety_reserve
        if self.agent_state.energy_remaining + 1e-9 < required:
            self.returning = True
            self._record("return_started", reason="energy_reserve")
        path = return_path
        if not self.returning:
            frontier_path = path_to_nearest_frontier(current, self.map.frontiers(), self.map.is_known_free)
            if frontier_path is None:
                self.returning = True
                self._record("return_started", reason="exploration_complete")
            else:
                path = frontier_path
        if self.returning:
            path = return_path
            if current == self.config.base:
                self.state = MissionBridgeState.COMPLETED
                self._record("mission_completed")
                self._capture_frame()
                return None
        if len(path) < 2:
            self._enter_error("planner_produced_no_step")
            return None
        self._command_counter += 1
        return MovementCommand(
            session_id=self.session_id,
            agent_id=self.agent_id,
            command_id=f"{self.session_id}:move:{self._command_counter}",
            expected_state_sequence=self.agent_state.sequence,
            issued_sim_time=self.agent_state.sim_time,
            valid_until_sim_time=self.agent_state.sim_time + self.config.command_validity_steps,
            source=current,
            target=path[1],
        )

    def authorize_resume(self, now_ns: int | None = None) -> bool:
        now = monotonic_ns() if now_ns is None else now_ns
        stamps = (self._last_observation_ns, self._last_state_ns, self._last_backend_heartbeat_ns)
        fresh = all(
            stamp is not None and now - stamp <= int(self.config.data_timeout_s * 1_000_000_000)
            for stamp in stamps
        )
        synchronized = (
            self.agent_state is not None
            and self._latest_observation_sequence == self._latest_state_sequence
            and self.pending_command is None
        )
        if self.state is not MissionBridgeState.CONTROLLED_STOP or not fresh or not synchronized:
            self._record("resume_rejected")
            return False
        self.error = ""
        self.state = MissionBridgeState.ACTIVE
        self._record("resume_authorized")
        return True

    def _capture_frame(self) -> None:
        if self.agent_state is None:
            return
        encoding = {CellState.UNKNOWN: "?", CellState.FREE: ".", CellState.OCCUPIED: "#"}
        occupancy = [
            "".join(encoding[self.map.cell_at(Position(x, y))] for x in range(self.config.width))
            for y in range(self.config.height)
        ]
        self._frames.append(
            {
                "step": self.agent_state.sequence,
                "position": [self.agent_state.position.x, self.agent_state.position.y],
                "state": self.state.value,
                "energy_remaining": round(self.agent_state.energy_remaining, 6),
                "returning": self.returning,
                "occupancy": occupancy,
                "confirmed_survivors": [[p.x, p.y] for p in sorted(self.confirmed_survivors)],
                "adapter": {
                    "pending_command_id": self.pending_command.command_id if self.pending_command else None,
                    "latest_observation_sequence": self._latest_observation_sequence,
                    "latest_state_sequence": self._latest_state_sequence,
                    "error": self.error or None,
                },
            }
        )

    def report(self) -> dict[str, object]:
        returned = self.agent_state is not None and self.agent_state.position == self.config.base
        return {
            "schema_version": "echorescue-closed-loop-report/1.0",
            "session_id": self.session_id,
            "agent_id": self.agent_id,
            "state": self.state.value,
            "completed": self.state is MissionBridgeState.COMPLETED,
            "returned_to_base": returned,
            "commands_issued": self._command_counter,
            "known_cells": self.map.known_cell_count,
            "explored_percent": round(self.map.explored_percent, 3),
            "survivors_expected": self.config.expected_survivors,
            "survivors_confirmed": len(self.confirmed_survivors),
            "mission_success": (
                self.state is MissionBridgeState.COMPLETED
                and returned
                and len(self.confirmed_survivors) >= self.config.expected_survivors
            ),
            "error": self.error or None,
            "events": list(self._events),
        }


class ClosedLoopSimulatorBackend:
    """The sole owner of world truth and execution-side safety checks."""

    def __init__(
        self,
        config: SimulationConfig,
        *,
        agent_id: str = "drone-1",
        command_lease_timeout_s: float = 2.0,
        clock_ns: Callable[[], int] = monotonic_ns,
    ) -> None:
        if config.drone_count != 1:
            raise ValueError("v0.13 backend supports exactly one agent and one floor")
        if command_lease_timeout_s <= 0:
            raise ValueError("command lease timeout must be positive")
        self.config = config
        self.agent_id = agent_id
        self.world = GridWorld.generate(config)
        self.position = self.world.base
        self.sensor = DistanceSensor(config.sensor_range)
        self.survivor_sensor = SurvivorSensor(
            max_range=config.active_survivor_sensor_range,
            channel=config.survivor_sensor,
            base_detection_probability=config.active_survivor_detection_probability,
            smoke_attenuation=config.active_survivor_smoke_attenuation,
        )
        self.battery = Battery(config.battery_capacity, config.movement_energy_cost, config.sensor_energy_cost)
        self.sim_time = 0.0
        self.state_sequence = 0
        self.active_session: str | None = None
        self.last_command_id = ""
        self.stopped = True
        self.error = ""
        self.shield_interventions = 0
        self._processed: dict[tuple[str, str], MovementResult] = {}
        self._last_mission_heartbeat_ns: int | None = None
        self._lease_ns = int(command_lease_timeout_s * 1_000_000_000)
        self._clock_ns = clock_ns
        self._heartbeat_sequence = 0

    def receive_mission_heartbeat(self, heartbeat: BridgeHeartbeat) -> bool:
        if heartbeat.component_id != "mission":
            return False
        now = self._clock_ns()
        lease_expired = self._lease_expired(now)
        if self.active_session not in {None, heartbeat.session_id} and not lease_expired:
            return False
        if self.active_session != heartbeat.session_id:
            self.active_session = heartbeat.session_id
            self.last_command_id = ""
            self.error = ""
        elif heartbeat.health is BridgeHealth.READY and self.error == "mission_heartbeat_timeout":
            self.error = ""
        self._last_mission_heartbeat_ns = now
        self.stopped = heartbeat.health in {BridgeHealth.ERROR, BridgeHealth.STOPPED}
        return True

    def _lease_expired(self, now_ns: int | None = None) -> bool:
        now = self._clock_ns() if now_ns is None else now_ns
        return self._last_mission_heartbeat_ns is None or now - self._last_mission_heartbeat_ns > self._lease_ns

    def watchdog(self) -> bool:
        if self._lease_expired():
            self.stopped = True
            self.error = "mission_heartbeat_timeout"
            return False
        return True

    def heartbeat(self) -> BridgeHeartbeat:
        self.watchdog()
        self._heartbeat_sequence += 1
        session = self.active_session or "unbound"
        health = BridgeHealth.ERROR if self.error else BridgeHealth.STOPPED if self.stopped else BridgeHealth.READY
        return BridgeHeartbeat(session, "simulator", self._heartbeat_sequence, self.sim_time, health, self.error)

    def state_report(self) -> AgentStateReport:
        if self.active_session is None:
            raise RuntimeError("backend has no active session")
        return AgentStateReport(
            self.active_session,
            self.agent_id,
            self.state_sequence,
            self.sim_time,
            GRID_FRAME,
            self.position,
            self.battery.remaining,
            self.last_command_id,
            "stopped" if self.stopped else "idle",
            self.error,
        )

    def observation(self) -> SensorObservation:
        if self.active_session is None:
            raise RuntimeError("backend has no active session")
        if not self.battery.consume(self.config.sensor_energy_cost):
            self.stopped = True
            self.error = "sensor_energy_exhausted"
        observed = self.sensor.observe(self.world, self.position)
        if self.config.uncertainty_profile != "off":
            observed = uncertain_observations(
                observed,
                seed=self.config.seed,
                agent_id=self.agent_id,
                step=self.state_sequence,
                profile=self.config.uncertainty_profile,
            )
        reliability = (
            0.95
            if self.config.uncertainty_profile == "off"
            else UNCERTAINTY_PROFILES[
                self.config.uncertainty_profile
            ].occupancy_reliability
        )
        survivor_report = self.survivor_sensor.observe_report(
            self.world,
            self.position,
            smoke_profile=self.config.smoke_profile,
            seed=self.config.seed,
            step=self.state_sequence,
            observer_id=self.agent_id,
            perception_noise=self.config.perception_noise,
        )
        survivor_evidence = tuple(
            SurvivorEvidence(item.position, item.confidence, True, survivor_report.sensor_channel)
            for item in survivor_report.observations
            if item.success
        )
        return SensorObservation(
            self.active_session,
            self.agent_id,
            f"obs-{self.state_sequence}",
            self.state_sequence,
            self.sim_time,
            GRID_FRAME,
            self.position,
            tuple(CellObservation(position, state, reliability) for position, state in sorted(observed.items())),
            survivor_evidence,
        )

    def execute(self, command: MovementCommand) -> MovementResult:
        key = (command.session_id, command.command_id)
        cached = self._processed.get(key)
        if cached is not None:
            return cached
        status = CommandStatus.REJECTED
        detail = ""
        if self.active_session != command.session_id or command.agent_id != self.agent_id:
            detail = "session_or_agent_mismatch"
        elif not self.watchdog() or self.stopped:
            detail = self.error or "backend_stopped"
        elif command.expected_state_sequence != self.state_sequence or command.source != self.position:
            detail = "state_precondition_failed"
        elif command.valid_until_sim_time < self.sim_time:
            detail = "command_expired"
        elif not self.world.is_free(command.target):
            self.shield_interventions += 1
            detail = "safety_shield_rejected_occupied_cell"
        elif not self.battery.consume(self.config.movement_energy_cost):
            self.stopped = True
            self.error = "movement_energy_exhausted"
            status = CommandStatus.FAILED
            detail = self.error
        else:
            # A grid step is atomic.  Once this branch starts it completes; there
            # is no claim of a continuous mid-cell emergency stop.
            self.position = command.target
            self.state_sequence += 1
            self.sim_time += 1.0
            self.last_command_id = command.command_id
            status = CommandStatus.COMPLETED
            detail = "atomic_grid_step_completed"
        result = MovementResult(
            command.session_id,
            self.agent_id,
            command.command_id,
            status,
            self.state_sequence,
            self.sim_time,
            self.position,
            detail,
        )
        self._processed[key] = result
        return result


def closed_loop_config(config: SimulationConfig) -> ClosedLoopConfig:
    return ClosedLoopConfig(
        width=config.width,
        height=config.height,
        base=Position(1, 1),
        expected_survivors=config.survivor_count,
        movement_energy_cost=config.movement_energy_cost,
        sensor_energy_cost=config.sensor_energy_cost,
        energy_safety_reserve=config.energy_safety_reserve,
        battery_capacity=config.battery_capacity,
    )
