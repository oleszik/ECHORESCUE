from dataclasses import dataclass, field
from enum import Enum
from hashlib import sha256

from echorescue.models import Position


@dataclass(frozen=True, slots=True)
class PerceptionNoiseProfile:
    name: str
    visual_false_negative_rate: float
    thermal_false_negative_rate: float
    visual_false_positive_rate: float
    thermal_false_positive_rate: float
    false_positive_confidence_min: float
    false_positive_confidence_max: float

    def false_negative_rate(self, channel: str) -> float:
        return (
            self.thermal_false_negative_rate
            if channel == "thermal"
            else self.visual_false_negative_rate
        )

    def false_positive_rate(self, channel: str) -> float:
        return (
            self.thermal_false_positive_rate
            if channel == "thermal"
            else self.visual_false_positive_rate
        )


PERCEPTION_NOISE_PROFILES = {
    "off": PerceptionNoiseProfile("off", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "moderate": PerceptionNoiseProfile(
        "moderate",
        visual_false_negative_rate=0.18,
        thermal_false_negative_rate=0.12,
        visual_false_positive_rate=0.08,
        thermal_false_positive_rate=0.05,
        false_positive_confidence_min=0.32,
        false_positive_confidence_max=0.52,
    ),
}


def deterministic_unit(*identity: object) -> float:
    payload = "|".join(str(part) for part in identity)
    return int.from_bytes(sha256(payload.encode("utf-8")).digest()[:8], "big") / 2**64


class HypothesisStatus(str, Enum):
    UNCONFIRMED = "unconfirmed"
    CONFIRMED = "confirmed"
    REJECTED = "rejected"


@dataclass(slots=True)
class SurvivorHypothesis:
    location: Position
    accumulated_evidence: float
    observation_count: int
    positive_observations: int
    negative_observations: int
    status: HypothesisStatus
    source_channels: set[str] = field(default_factory=set)
    source_agents: set[str] = field(default_factory=set)
    first_observed_step: int = 0
    last_observed_step: int = 0
    confirmation_confidence: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "location": [self.location.x, self.location.y],
            "accumulated_evidence": round(self.accumulated_evidence, 6),
            "observation_count": self.observation_count,
            "positive_observations": self.positive_observations,
            "negative_observations": self.negative_observations,
            "status": self.status.value,
            "source_channels": sorted(self.source_channels),
            "source_agents": sorted(self.source_agents),
            "first_observed_step": self.first_observed_step,
            "last_observed_step": self.last_observed_step,
            "confirmation_confidence": (
                round(self.confirmation_confidence, 6)
                if self.confirmation_confidence is not None
                else None
            ),
        }


@dataclass(frozen=True, slots=True)
class EvidenceUpdate:
    location: Position
    evidence_before: float
    evidence_after: float
    observation_count: int
    status_before: HypothesisStatus | None
    status_after: HypothesisStatus
    created: bool
    confirmed: bool
    rejected: bool


class SurvivorHypothesisTracker:
    """Accumulate location-keyed evidence without access to Ground Truth."""

    def __init__(
        self,
        *,
        minimum_positive_observations: int,
        confirmation_threshold: float,
        rejection_threshold: float,
        negative_evidence_weight: float,
    ) -> None:
        self.minimum_positive_observations = minimum_positive_observations
        self.confirmation_threshold = confirmation_threshold
        self.rejection_threshold = rejection_threshold
        self.negative_evidence_weight = negative_evidence_weight
        self.hypotheses: dict[Position, SurvivorHypothesis] = {}

    def positive(
        self,
        location: Position,
        *,
        confidence: float,
        channel: str,
        agent_id: str,
        step: int,
    ) -> EvidenceUpdate:
        confidence = min(1.0, max(0.0, confidence))
        hypothesis = self.hypotheses.get(location)
        created = hypothesis is None
        if hypothesis is None:
            hypothesis = SurvivorHypothesis(
                location=location,
                accumulated_evidence=0.0,
                observation_count=0,
                positive_observations=0,
                negative_observations=0,
                status=HypothesisStatus.UNCONFIRMED,
                first_observed_step=step,
                last_observed_step=step,
            )
            self.hypotheses[location] = hypothesis
        status_before = None if created else hypothesis.status
        evidence_before = hypothesis.accumulated_evidence
        if hypothesis.status is HypothesisStatus.UNCONFIRMED:
            hypothesis.accumulated_evidence = min(
                1.0, hypothesis.accumulated_evidence + confidence * 0.5
            )
            hypothesis.observation_count += 1
            hypothesis.positive_observations += 1
            hypothesis.source_channels.add(channel)
            hypothesis.source_agents.add(agent_id)
            hypothesis.last_observed_step = step
            if (
                hypothesis.positive_observations
                >= self.minimum_positive_observations
                and hypothesis.accumulated_evidence >= self.confirmation_threshold
            ):
                hypothesis.status = HypothesisStatus.CONFIRMED
                hypothesis.confirmation_confidence = (
                    hypothesis.accumulated_evidence
                )
        return EvidenceUpdate(
            location=location,
            evidence_before=evidence_before,
            evidence_after=hypothesis.accumulated_evidence,
            observation_count=hypothesis.observation_count,
            status_before=status_before,
            status_after=hypothesis.status,
            created=created,
            confirmed=(
                status_before is not HypothesisStatus.CONFIRMED
                and hypothesis.status is HypothesisStatus.CONFIRMED
            ),
            rejected=False,
        )

    def negative(
        self,
        location: Position,
        *,
        confidence: float,
        channel: str,
        agent_id: str,
        step: int,
    ) -> EvidenceUpdate | None:
        hypothesis = self.hypotheses.get(location)
        if hypothesis is None or hypothesis.status is not HypothesisStatus.UNCONFIRMED:
            return None
        confidence = min(1.0, max(0.0, confidence))
        evidence_before = hypothesis.accumulated_evidence
        status_before = hypothesis.status
        hypothesis.accumulated_evidence = max(
            0.0,
            hypothesis.accumulated_evidence
            - confidence * self.negative_evidence_weight,
        )
        hypothesis.observation_count += 1
        hypothesis.negative_observations += 1
        hypothesis.source_channels.add(channel)
        hypothesis.source_agents.add(agent_id)
        hypothesis.last_observed_step = step
        if (
            hypothesis.negative_observations >= 2
            and hypothesis.accumulated_evidence <= self.rejection_threshold
        ):
            hypothesis.status = HypothesisStatus.REJECTED
        return EvidenceUpdate(
            location=location,
            evidence_before=evidence_before,
            evidence_after=hypothesis.accumulated_evidence,
            observation_count=hypothesis.observation_count,
            status_before=status_before,
            status_after=hypothesis.status,
            created=False,
            confirmed=False,
            rejected=hypothesis.status is HypothesisStatus.REJECTED,
        )

    @property
    def confirmed_locations(self) -> set[Position]:
        return {
            location
            for location, hypothesis in self.hypotheses.items()
            if hypothesis.status is HypothesisStatus.CONFIRMED
        }

