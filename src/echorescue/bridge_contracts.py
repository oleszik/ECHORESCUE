"""ROS-independent contracts for the v0.13 closed-loop bridge.

The dataclasses in this module are the canonical wire-domain model.  ROS
messages are adapters for these values; importing EchoRescue never imports
``rclpy`` or generated ROS interfaces.
"""

from dataclasses import dataclass
from enum import Enum
from math import floor, isfinite
import re

from echorescue.models import CellState, Position


GRID_FRAME = "echorescue/grid"
MAP_FRAME = "map"
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")


def _require_identifier(value: str, field: str) -> None:
    if not _IDENTIFIER.fullmatch(value):
        raise ValueError(f"{field} is not a valid identifier")


def _require_non_negative(value: int | float, field: str) -> None:
    if not isfinite(float(value)) or value < 0:
        raise ValueError(f"{field} must be finite and non-negative")


class CommandStatus(str, Enum):
    ACCEPTED = "accepted"
    COMPLETED = "completed"
    REJECTED = "rejected"
    CANCELED = "canceled"
    FAILED = "failed"


class BridgeHealth(str, Enum):
    STARTING = "starting"
    READY = "ready"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class GridCoordinateMapper:
    """Map discrete cells to ENU metres at cell centres."""

    cell_size_m: float = 1.0
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    origin_z_m: float = 0.0
    frame_id: str = MAP_FRAME

    def __post_init__(self) -> None:
        if not isfinite(self.cell_size_m) or self.cell_size_m <= 0:
            raise ValueError("cell_size_m must be finite and positive")
        for value in (self.origin_x_m, self.origin_y_m, self.origin_z_m):
            if not isfinite(value):
                raise ValueError("coordinate origin must be finite")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")

    def grid_to_metric(self, position: Position) -> tuple[float, float, float]:
        return (
            self.origin_x_m + (position.x + 0.5) * self.cell_size_m,
            self.origin_y_m + (position.y + 0.5) * self.cell_size_m,
            self.origin_z_m,
        )

    def metric_to_grid(self, x_m: float, y_m: float) -> Position:
        if not isfinite(x_m) or not isfinite(y_m):
            raise ValueError("metric coordinates must be finite")
        return Position(
            floor((x_m - self.origin_x_m) / self.cell_size_m),
            floor((y_m - self.origin_y_m) / self.cell_size_m),
        )


@dataclass(frozen=True, slots=True)
class CellObservation:
    position: Position
    state: CellState
    confidence: float

    def __post_init__(self) -> None:
        if self.state is CellState.UNKNOWN:
            raise ValueError("sensor observations cannot assert unknown")
        if not isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("cell confidence must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class SurvivorEvidence:
    position: Position
    confidence: float
    detected: bool = True
    channel: str = "visual"

    def __post_init__(self) -> None:
        if not isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("survivor confidence must be in [0, 1]")
        if not self.channel:
            raise ValueError("survivor channel must not be empty")


@dataclass(frozen=True, slots=True)
class SensorObservation:
    session_id: str
    agent_id: str
    observation_id: str
    sequence: int
    sim_time: float
    frame_id: str
    agent_position: Position
    cells: tuple[CellObservation, ...]
    survivors: tuple[SurvivorEvidence, ...] = ()

    def __post_init__(self) -> None:
        _require_identifier(self.session_id, "session_id")
        _require_identifier(self.agent_id, "agent_id")
        _require_identifier(self.observation_id, "observation_id")
        _require_non_negative(self.sequence, "sequence")
        _require_non_negative(self.sim_time, "sim_time")
        if self.frame_id != GRID_FRAME:
            raise ValueError(f"frame_id must be {GRID_FRAME!r}")
        positions = [cell.position for cell in self.cells]
        if len(positions) != len(set(positions)):
            raise ValueError("an observation may contain each cell only once")


@dataclass(frozen=True, slots=True)
class AgentStateReport:
    session_id: str
    agent_id: str
    sequence: int
    sim_time: float
    frame_id: str
    position: Position
    energy_remaining: float
    last_command_id: str = ""
    motion_state: str = "idle"
    error: str = ""

    def __post_init__(self) -> None:
        _require_identifier(self.session_id, "session_id")
        _require_identifier(self.agent_id, "agent_id")
        _require_non_negative(self.sequence, "sequence")
        _require_non_negative(self.sim_time, "sim_time")
        _require_non_negative(self.energy_remaining, "energy_remaining")
        if self.frame_id != GRID_FRAME:
            raise ValueError(f"frame_id must be {GRID_FRAME!r}")
        if self.last_command_id:
            _require_identifier(self.last_command_id, "last_command_id")


@dataclass(frozen=True, slots=True)
class MovementCommand:
    session_id: str
    agent_id: str
    command_id: str
    expected_state_sequence: int
    issued_sim_time: float
    valid_until_sim_time: float
    source: Position
    target: Position

    def __post_init__(self) -> None:
        _require_identifier(self.session_id, "session_id")
        _require_identifier(self.agent_id, "agent_id")
        _require_identifier(self.command_id, "command_id")
        _require_non_negative(self.expected_state_sequence, "expected_state_sequence")
        _require_non_negative(self.issued_sim_time, "issued_sim_time")
        _require_non_negative(self.valid_until_sim_time, "valid_until_sim_time")
        if self.valid_until_sim_time <= self.issued_sim_time:
            raise ValueError("command validity must extend beyond issue time")
        if abs(self.source.x - self.target.x) + abs(self.source.y - self.target.y) != 1:
            raise ValueError("movement command must be one cardinal cell step")


@dataclass(frozen=True, slots=True)
class MovementResult:
    session_id: str
    agent_id: str
    command_id: str
    status: CommandStatus
    state_sequence: int
    sim_time: float
    actual_position: Position
    detail: str = ""

    def __post_init__(self) -> None:
        _require_identifier(self.session_id, "session_id")
        _require_identifier(self.agent_id, "agent_id")
        _require_identifier(self.command_id, "command_id")
        _require_non_negative(self.state_sequence, "state_sequence")
        _require_non_negative(self.sim_time, "sim_time")


@dataclass(frozen=True, slots=True)
class BridgeHeartbeat:
    session_id: str
    component_id: str
    sequence: int
    sim_time: float
    health: BridgeHealth
    error: str = ""

    def __post_init__(self) -> None:
        _require_identifier(self.session_id, "session_id")
        _require_identifier(self.component_id, "component_id")
        _require_non_negative(self.sequence, "sequence")
        _require_non_negative(self.sim_time, "sim_time")
        if self.health is BridgeHealth.ERROR and not self.error:
            raise ValueError("error health requires an error description")
