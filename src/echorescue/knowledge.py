from dataclasses import dataclass
from typing import Iterable

from echorescue.models import CellState, Position


STATE_PRIORITY = {
    CellState.FREE: 1,
    CellState.OCCUPIED: 2,
}


@dataclass(frozen=True, slots=True)
class CellKnowledge:
    state: CellState
    observed_step: int
    source_id: str


def merge_cell_knowledge(
    current: CellKnowledge | None,
    incoming: CellKnowledge,
) -> CellKnowledge:
    """Merge without ground truth using a stable safety-first ordering."""

    if current is None:
        return incoming
    current_key = (
        STATE_PRIORITY[current.state],
        current.observed_step,
        current.source_id,
    )
    incoming_key = (
        STATE_PRIORITY[incoming.state],
        incoming.observed_step,
        incoming.source_id,
    )
    return incoming if incoming_key > current_key else current


class KnowledgeMap:
    """Operator-safe local knowledge with observation age and provenance."""

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self._records: dict[Position, CellKnowledge] = {}

    def contains(self, position: Position) -> bool:
        return 0 <= position.x < self.width and 0 <= position.y < self.height

    def cell_at(self, position: Position) -> CellState:
        record = self._records.get(position)
        return record.state if record is not None else CellState.UNKNOWN

    def record_at(self, position: Position) -> CellKnowledge | None:
        return self._records.get(position)

    def is_known_free(self, position: Position) -> bool:
        return self.cell_at(position) is CellState.FREE

    def frontiers(self) -> tuple[Position, ...]:
        result = []
        for y in range(self.height):
            for x in range(self.width):
                position = Position(x, y)
                if self.is_known_free(position) and any(
                    self.contains(neighbor)
                    and self.cell_at(neighbor) is CellState.UNKNOWN
                    for neighbor in position.neighbors()
                ):
                    result.append(position)
        return tuple(result)

    @property
    def records(self) -> tuple[tuple[Position, CellKnowledge], ...]:
        return tuple(sorted(self._records.items()))

    def observe(
        self,
        observations: dict[Position, CellState],
        *,
        step: int,
        source_id: str,
    ) -> tuple[Position, ...]:
        incoming = (
            (
                position,
                CellKnowledge(
                    state=state,
                    observed_step=step,
                    source_id=source_id,
                ),
            )
            for position, state in sorted(observations.items())
            if state is not CellState.UNKNOWN and self.contains(position)
        )
        return self.apply(incoming)

    def apply(
        self,
        records: Iterable[tuple[Position, CellKnowledge]],
    ) -> tuple[Position, ...]:
        changed = []
        for position, incoming in sorted(records):
            if not self.contains(position):
                continue
            current = self._records.get(position)
            merged = merge_cell_knowledge(current, incoming)
            if merged != current:
                self._records[position] = merged
                changed.append(position)
        return tuple(changed)

    @property
    def known_cell_count(self) -> int:
        return len(self._records)

    @property
    def known_coverage(self) -> float:
        return 100.0 * self.known_cell_count / (self.width * self.height)

    def differs_from(self, other: "KnowledgeMap") -> tuple[Position, ...]:
        return tuple(
            Position(x, y)
            for y in range(self.height)
            for x in range(self.width)
            if (
                self.cell_at(Position(x, y))
                is not other.cell_at(Position(x, y))
            )
        )

    def stale_against(self, reference: "KnowledgeMap") -> tuple[Position, ...]:
        return tuple(
            position
            for position, record in reference.records
            if self.record_at(position) != record
        )

    def average_data_age(self, step: int) -> float:
        if not self._records:
            return 0.0
        return sum(
            max(0, step - record.observed_step)
            for record in self._records.values()
        ) / len(self._records)

    def oldest_data_age(self, step: int) -> int:
        return max(
            (
                max(0, step - record.observed_step)
                for record in self._records.values()
            ),
            default=0,
        )


def merge_knowledge_maps(maps: Iterable[KnowledgeMap]) -> KnowledgeMap:
    maps = tuple(maps)
    if not maps:
        raise ValueError("at least one knowledge map is required")
    merged = KnowledgeMap(maps[0].width, maps[0].height)
    for knowledge_map in maps:
        if (
            knowledge_map.width != merged.width
            or knowledge_map.height != merged.height
        ):
            raise ValueError("knowledge maps must have matching dimensions")
        merged.apply(knowledge_map.records)
    return merged
