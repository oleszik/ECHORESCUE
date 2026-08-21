from dataclasses import dataclass

from echorescue.environment import GridWorld
from echorescue.models import DroneStatus, Position


@dataclass(frozen=True, slots=True)
class MotionIntent:
    drone_id: str
    current_position: Position
    next_position: Position
    reservation: tuple[Position, ...]
    status: DroneStatus
    energy_remaining: float
    safe_energy_margin: float
    valid_until_step: int


@dataclass(frozen=True, slots=True)
class IntentConflict:
    kind: str
    position: Position
    drone_ids: tuple[str, str]


@dataclass(frozen=True, slots=True)
class ProximitySensor:
    max_range: int

    def __post_init__(self) -> None:
        if self.max_range < 1:
            raise ValueError("proximity range must be positive")

    def can_detect(
        self,
        world: GridWorld,
        observer: Position,
        other: Position,
    ) -> bool:
        if (
            abs(observer.x - other.x) + abs(observer.y - other.y)
            > self.max_range
        ):
            return False
        frontier = {observer}
        visited = {observer}
        for _ in range(self.max_range):
            next_frontier: set[Position] = set()
            for position in frontier:
                for neighbor in position.neighbors():
                    if neighbor == other:
                        return True
                    if neighbor in visited or not world.is_free(neighbor):
                        continue
                    visited.add(neighbor)
                    next_frontier.add(neighbor)
            frontier = next_frontier
            if not frontier:
                break
        return observer == other


def _reservation(intent: MotionIntent) -> tuple[Position, ...]:
    return intent.reservation or (intent.next_position,)


def detect_intent_conflict(
    first: MotionIntent,
    second: MotionIntent,
    base: Position,
) -> IntentConflict | None:
    drone_ids = tuple(sorted((first.drone_id, second.drone_id)))
    if (
        first.next_position == second.next_position
        and first.next_position != base
        and (
            first.next_position != first.current_position
            or second.next_position != second.current_position
        )
    ):
        return IntentConflict("vertex", first.next_position, drone_ids)
    if (
        first.next_position == second.current_position
        and second.next_position == first.current_position
        and first.current_position != second.current_position
    ):
        return IntentConflict("edge_swap", first.next_position, drone_ids)
    if (
        first.next_position == second.current_position
        and first.current_position != second.current_position
    ):
        return IntentConflict(
            "occupied_current_cell", first.next_position, drone_ids
        )
    if (
        second.next_position == first.current_position
        and first.current_position != second.current_position
    ):
        return IntentConflict(
            "occupied_current_cell", second.next_position, drone_ids
        )

    first_path = _reservation(first)
    second_path = _reservation(second)
    for first_position, second_position in zip(first_path, second_path):
        if first_position == second_position and first_position != base:
            return IntentConflict(
                "reservation_vertex", first_position, drone_ids
            )

    first_edges = set(
        zip((first.current_position,) + first_path, first_path)
    )
    second_edges = set(
        zip((second.current_position,) + second_path, second_path)
    )
    opposing_edges = sorted(
        (start, end)
        for start, end in first_edges
        if (end, start) in second_edges and start != end
    )
    if opposing_edges:
        return IntentConflict(
            "corridor_head_on", opposing_edges[0][1], drone_ids
        )
    return None


def proximity_risk(
    intent: MotionIntent,
    other_position: Position,
) -> str | None:
    if intent.next_position == intent.current_position:
        return None
    if intent.next_position == other_position:
        return "proximity_occupied"
    current_distance = (
        abs(intent.current_position.x - other_position.x)
        + abs(intent.current_position.y - other_position.y)
    )
    next_distance = (
        abs(intent.next_position.x - other_position.x)
        + abs(intent.next_position.y - other_position.y)
    )
    if next_distance <= 1 and next_distance < current_distance:
        return "proximity_convergence"
    return None


def priority_key(
    intent: MotionIntent,
    consecutive_yield_steps: int,
) -> tuple[object, ...]:
    urgent = (
        intent.status is DroneStatus.RETURN_HOME
        or intent.safe_energy_margin <= 0.0
    )
    return (
        0 if urgent else 1,
        intent.safe_energy_margin,
        -consecutive_yield_steps,
        intent.drone_id,
    )
