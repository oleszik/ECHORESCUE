from dataclasses import dataclass
from enum import Enum

from echorescue.models import Position


class EventType(str, Enum):
    SURVIVOR_DETECTED = "survivor_detected"
    SURVIVOR_CONFIRMED = "survivor_confirmed"
    RETURN_STARTED = "return_started"
    BASE_REACHED = "base_reached"
    ENERGY_EMERGENCY = "energy_emergency"
    RETURN_PATH_UNAVAILABLE = "return_path_unavailable"
    RETURN_REPLANNED = "return_replanned"


@dataclass(frozen=True, slots=True)
class MissionEvent:
    position: Position
    step: int
    drone_id: str
    event_type: EventType
    energy_remaining: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "position": [self.position.x, self.position.y],
            "step": self.step,
            "drone_id": self.drone_id,
            "event_type": self.event_type.value,
            "energy_remaining": (
                round(self.energy_remaining, 6)
                if self.energy_remaining is not None
                else None
            ),
        }


class MissionLog:
    """Append-only event log with transition-aware deduplication."""

    def __init__(self) -> None:
        self._events: list[MissionEvent] = []
        self._event_keys: set[tuple[object, ...]] = set()

    @property
    def events(self) -> tuple[MissionEvent, ...]:
        return tuple(self._events)

    def record(self, event: MissionEvent) -> bool:
        if event.event_type in {
            EventType.SURVIVOR_DETECTED,
            EventType.SURVIVOR_CONFIRMED,
        }:
            key: tuple[object, ...] = (event.event_type, event.position)
        elif event.event_type is EventType.RETURN_REPLANNED:
            key = (event.event_type, event.step, event.position)
        else:
            key = (event.event_type,)
        if key in self._event_keys:
            return False
        self._event_keys.add(key)
        self._events.append(event)
        return True
