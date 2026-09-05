"""Bounded, observation-based occupancy evidence (simulation time only)."""
from dataclasses import dataclass
from math import exp, log, log2, fsum

from echorescue.models import CellState


@dataclass(frozen=True, slots=True)
class UncertaintyProfile:
    occupancy_detection: float
    occupancy_false_positive: float
    occupancy_reliability: float
    survivor_detection: float
    survivor_false_positive: float
    survivor_reliability: float


UNCERTAINTY_PROFILES = {
    "clean": UncertaintyProfile(1.0, 0.0, .95, 1.0, 0.0, .95),
    "low_noise": UncertaintyProfile(.95, .02, .90, .95, .02, .90),
    "medium_noise": UncertaintyProfile(.85, .08, .80, .85, .08, .80),
    "high_noise": UncertaintyProfile(.65, .20, .65, .65, .20, .65),
}


@dataclass(frozen=True, slots=True)
class ProbabilityConfig:
    prior: float = .5
    free_threshold: float = .35
    occupied_threshold: float = .65
    max_log_odds: float = 4.0
    half_life: float = 80.0
    retention_steps: int = 640

    def __post_init__(self) -> None:
        if not 0 < self.free_threshold < self.prior < self.occupied_threshold < 1:
            raise ValueError("thresholds must bracket an interior prior")
        if self.max_log_odds <= 0 or self.half_life <= 0 or self.retention_steps < 1:
            raise ValueError("bounds, half life and retention must be positive")


@dataclass(frozen=True, order=True, slots=True)
class OccupancyEvidence:
    observed_step: int
    source_id: str
    floor: int
    occupied: bool
    reliability: float

    @property
    def identity(self) -> tuple[int, str, int]:
        # Position is the enclosing cell key. One range sample per agent/tick/cell.
        return self.observed_step, self.source_id, self.floor

    def __post_init__(self) -> None:
        if not .5 < self.reliability < 1 or self.observed_step < 0:
            raise ValueError("evidence requires finite reliability in (.5, 1) and time >= 0")


def union_evidence(*groups: tuple[OccupancyEvidence, ...]) -> tuple[OccupancyEvidence, ...]:
    unique: dict[tuple[int, str, int], OccupancyEvidence] = {}
    for group in groups:
        for item in group:
            previous = unique.get(item.identity)
            if previous is not None and previous != item:
                raise ValueError("conflicting payload for the same observation identity")
            unique[item.identity] = item
    return tuple(sorted(unique.values()))


@dataclass(frozen=True, slots=True)
class ProbabilisticCellState:
    probability: float
    log_odds: float
    state: CellState
    observed_step: int | None
    sources: tuple[str, ...]
    evidence_count: int


def evaluate(evidence: tuple[OccupancyEvidence, ...], step: int,
             config: ProbabilityConfig) -> ProbabilisticCellState:
    active = tuple(e for e in evidence if 0 <= step-e.observed_step < config.retention_steps)
    prior_odds = log(config.prior / (1-config.prior))
    # Bound each timestamp's aggregate before decay. Reading never changes evidence.
    grouped: dict[int, list[float]] = {}
    for e in active:
        grouped.setdefault(e.observed_step, []).append(
            (1 if e.occupied else -1) * log(e.reliability/(1-e.reliability)))
    value = prior_odds
    previous = min(grouped, default=step)
    for timestamp, values in sorted(grouped.items()):
        value = prior_odds + (value-prior_odds) * 2**(-(timestamp-previous)/config.half_life)
        value = max(-config.max_log_odds, min(config.max_log_odds, value+fsum(values)))
        previous = timestamp
    value = prior_odds + (value-prior_odds) * 2**(-(step-previous)/config.half_life)
    probability = 1/(1+exp(-value))
    state = (CellState.FREE if probability <= config.free_threshold else
             CellState.OCCUPIED if probability >= config.occupied_threshold else CellState.UNKNOWN)
    return ProbabilisticCellState(probability, value, state,
        max((e.observed_step for e in active), default=None),
        tuple(sorted({e.source_id for e in active})), len(active))


def entropy(probability: float) -> float:
    return -probability*log2(probability)-(1-probability)*log2(1-probability)
