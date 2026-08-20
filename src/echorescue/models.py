from dataclasses import dataclass
from enum import Enum


class CellState(str, Enum):
    UNKNOWN = "unknown"
    FREE = "free"
    OCCUPIED = "occupied"


class DroneStatus(str, Enum):
    EXPLORE = "EXPLORE"
    RETURN_HOME = "RETURN_HOME"
    LANDED = "LANDED"
    ENERGY_EMERGENCY = "ENERGY_EMERGENCY"
    RETURN_PATH_UNAVAILABLE = "RETURN_PATH_UNAVAILABLE"
    FAILED = "FAILED"


@dataclass(frozen=True, order=True, slots=True)
class Position:
    x: int
    y: int

    def neighbors(self) -> tuple["Position", ...]:
        """Return cardinal neighbors in a stable order."""

        return (
            Position(self.x + 1, self.y),
            Position(self.x, self.y + 1),
            Position(self.x - 1, self.y),
            Position(self.x, self.y - 1),
        )


@dataclass(slots=True)
class Drone:
    position: Position
    identifier: str = "drone-1"
    path_length: int = 0
    status: DroneStatus = DroneStatus.EXPLORE
