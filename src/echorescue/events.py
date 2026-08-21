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
    FRONTIER_ASSIGNED = "frontier_assigned"
    FRONTIER_REASSIGNED = "frontier_reassigned"
    MOVEMENT_CONFLICT = "movement_conflict"
    DRONE_WAITED = "drone_waited"
    COMMUNICATION_LOST = "communication_lost"
    COMMUNICATION_RESTORED = "communication_restored"
    RELAY_LINK_ESTABLISHED = "relay_link_established"
    RELAY_LINK_LOST = "relay_link_lost"
    MAP_SYNC_STARTED = "map_sync_started"
    MAP_CELLS_UPLOADED = "map_cells_uploaded"
    MAP_CELLS_RECEIVED = "map_cells_received"
    MAP_CONVERGED = "map_converged"
    KNOWLEDGE_MODE_ACTIVATED = "knowledge_mode_activated"
    LOCAL_FRONTIER_SELECTED = "local_frontier_selected"
    STALE_TARGET_DISCARDED = "stale_target_discarded"
    SURVIVOR_KNOWLEDGE_SYNCHRONIZED = "survivor_knowledge_synchronized"
    SAFETY_SHIELD_INTERVENTION = "safety_shield_intervention"


@dataclass(frozen=True, slots=True)
class MissionEvent:
    position: Position
    step: int
    drone_id: str
    event_type: EventType
    energy_remaining: float | None = None
    cell_count: int | None = None

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
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
        if self.cell_count is not None:
            payload["cell_count"] = self.cell_count
        return payload


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
            key: tuple[object, ...] = (
                event.event_type,
                event.drone_id,
                event.position,
            )
        elif event.event_type in {
            EventType.RETURN_REPLANNED,
            EventType.FRONTIER_ASSIGNED,
            EventType.FRONTIER_REASSIGNED,
            EventType.MOVEMENT_CONFLICT,
            EventType.DRONE_WAITED,
            EventType.COMMUNICATION_LOST,
            EventType.COMMUNICATION_RESTORED,
            EventType.RELAY_LINK_ESTABLISHED,
            EventType.RELAY_LINK_LOST,
            EventType.MAP_SYNC_STARTED,
            EventType.MAP_CELLS_UPLOADED,
            EventType.MAP_CELLS_RECEIVED,
            EventType.MAP_CONVERGED,
            EventType.KNOWLEDGE_MODE_ACTIVATED,
            EventType.LOCAL_FRONTIER_SELECTED,
            EventType.STALE_TARGET_DISCARDED,
            EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED,
            EventType.SAFETY_SHIELD_INTERVENTION,
        }:
            key = (event.event_type, event.drone_id, event.step, event.position)
        else:
            key = (event.event_type, event.drone_id)
        if key in self._event_keys:
            return False
        self._event_keys.add(key)
        self._events.append(event)
        return True
