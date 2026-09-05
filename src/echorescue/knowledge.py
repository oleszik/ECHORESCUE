from dataclasses import dataclass
from typing import Iterable

from echorescue.models import CellState, Position
from echorescue.probabilistic import (OccupancyEvidence, ProbabilityConfig,
    ProbabilisticCellState, evaluate, union_evidence, entropy)


STATE_PRIORITY = {
    CellState.FREE: 1,
    CellState.OCCUPIED: 2,
}


@dataclass(frozen=True, slots=True)
class CellKnowledge:
    state: CellState
    observed_step: int
    source_id: str
    evidence: tuple[OccupancyEvidence, ...] = ()


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
    first = maps[0]
    merged: KnowledgeMap
    if isinstance(first, ProbabilisticKnowledgeMap):
        merged = ProbabilisticKnowledgeMap(first.width, first.height,
            probability_config=first.probability_config, reliability=first.reliability,
            floor=first.floor, planning_variant=first.planning_variant)
        merged.advance(max(m.step for m in maps if isinstance(m, ProbabilisticKnowledgeMap)))
    else:
        merged = KnowledgeMap(first.width, first.height)
    for knowledge_map in maps:
        if (
            knowledge_map.width != merged.width
            or knowledge_map.height != merged.height
        ):
            raise ValueError("knowledge maps must have matching dimensions")
        merged.apply(knowledge_map.records)
    return merged


class ProbabilisticKnowledgeMap(KnowledgeMap):
    """Same knowledge/transport interface; union raw evidence, never fused odds."""

    def __init__(self, width: int, height: int, *,
                 probability_config: ProbabilityConfig = ProbabilityConfig(),
                 reliability: float = .95, floor: int = 0,
                 planning_variant: str = "naive") -> None:
        super().__init__(width, height)
        self.probability_config = probability_config
        self.reliability = reliability
        self.floor = floor
        self.planning_variant = planning_variant
        self.step = 0
        self._probability_cache: dict[Position, ProbabilisticCellState] = {}

    def advance(self, step: int) -> None:
        if step < self.step:
            raise ValueError("simulation time cannot go backwards")
        if step == self.step:
            return
        self.step = step
        self._probability_cache.clear()
        for position, record in list(self._records.items()):
            retained = tuple(e for e in record.evidence
                             if 0 <= step-e.observed_step < self.probability_config.retention_steps)
            if retained:
                self._records[position] = self._record(retained)
            else:
                del self._records[position]

    def _record(self, evidence: tuple[OccupancyEvidence, ...]) -> CellKnowledge:
        state = evaluate(evidence, self.step, self.probability_config)
        return CellKnowledge(state.state, max(e.observed_step for e in evidence),
                             min(e.source_id for e in evidence), evidence)

    def probability_at(self, position: Position) -> ProbabilisticCellState:
        if position not in self._probability_cache:
            record = self._records.get(position)
            self._probability_cache[position] = evaluate(
                record.evidence if record else (), self.step, self.probability_config)
        return self._probability_cache[position]

    def cell_at(self, position: Position) -> CellState:
        if not self.contains(position):
            return CellState.OCCUPIED
        return self.probability_at(position).state

    def observe(self, observations: dict[Position, CellState], *, step: int,
                source_id: str) -> tuple[Position, ...]:
        self.advance(step)
        return self.apply((position, CellKnowledge(state, step, source_id,
            (OccupancyEvidence(step, source_id, self.floor, state is CellState.OCCUPIED,
                               self.reliability),)))
            for position, state in observations.items() if state is not CellState.UNKNOWN)

    def apply(self, records: Iterable[tuple[Position, CellKnowledge]]) -> tuple[Position, ...]:
        changed = set()
        for position, incoming in records:
            if not self.contains(position):
                continue
            current = self._records.get(position)
            evidence = union_evidence(current.evidence if current else (), incoming.evidence)
            evidence = tuple(e for e in evidence if e.floor == self.floor and
                             0 <= self.step-e.observed_step < self.probability_config.retention_steps)
            if evidence:
                record = self._record(evidence)
                if current != record:
                    self._records[position] = record
                    self._probability_cache.pop(position, None)
                    changed.add(position)
        return tuple(sorted(changed))

    @property
    def explored_percent(self) -> float:
        return self.known_coverage

    def information_bonus(self, target: Position) -> float:
        if self.planning_variant != "uncertainty-aware":
            return 0.0
        neighbors = [p for p in target.neighbors() if self.contains(p)]
        return 4.0 * sum(entropy(self.probability_at(p).probability) for p in neighbors)/4

    def telemetry(self) -> list[dict[str, object]]:
        result = []
        for position in sorted(self._records):
            state = self.probability_at(position)
            result.append({"position": [position.x, position.y], "floor": self.floor,
                "probability": round(state.probability, 6),
                "classification": "uncertain" if state.state is CellState.UNKNOWN else state.state.value,
                "age": self.step-state.observed_step if state.observed_step is not None else None,
                "sources": list(state.sources), "evidence_count": state.evidence_count})
        return result
