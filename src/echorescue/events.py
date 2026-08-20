from dataclasses import dataclass
from enum import Enum

from echorescue.models import Position


class EventType(str, Enum):
    SURVIVOR_DETECTED = "survivor_detected"
    SURVIVOR_CONFIRMED = "survivor_confirmed"


@dataclass(frozen=True, slots=True)
class MissionEvent:
    position: Position
    step: int
    drone_id: str
    event_type: EventType

    def to_dict(self) -> dict[str, object]:
        return {
            "position": [self.position.x, self.position.y],
            "step": self.step,
            "drone_id": self.drone_id,
            "event_type": self.event_type.value,
        }


class MissionLog:
    """Append-only event log with survivor-event deduplication."""

    def __init__(self) -> None:
        self._events: list[MissionEvent] = []
        self._survivor_event_keys: set[tuple[EventType, Position]] = set()

    @property
    def events(self) -> tuple[MissionEvent, ...]:
        return tuple(self._events)

    def record(self, event: MissionEvent) -> bool:
        key = (event.event_type, event.position)
        if key in self._survivor_event_keys:
            return False
        self._survivor_event_keys.add(key)
        self._events.append(event)
        return True

