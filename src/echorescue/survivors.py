from dataclasses import dataclass
from hashlib import sha256
from math import sqrt

from echorescue.environment import GridWorld
from echorescue.models import Position
from echorescue.perception import PERCEPTION_NOISE_PROFILES, deterministic_unit


def _line_cells(start: Position, end: Position) -> tuple[Position, ...]:
    """Return a conservative supercover line between two grid cells."""

    x, y = start.x, start.y
    dx = end.x - start.x
    dy = end.y - start.y
    nx = abs(dx)
    ny = abs(dy)
    sign_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    sign_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    ix = 0
    iy = 0
    cells = [start]

    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            # At a corner crossing, both adjacent cells are part of the
            # conservative visibility corridor. Either may block the ray.
            cells.append(Position(x + sign_x, y))
            cells.append(Position(x, y + sign_y))
            x += sign_x
            y += sign_y
            ix += 1
            iy += 1
        elif decision < 0:
            x += sign_x
            ix += 1
        else:
            y += sign_y
            iy += 1
        cells.append(Position(x, y))
    return tuple(dict.fromkeys(cells))


@dataclass(frozen=True, slots=True)
class SurvivorSensor:
    """Ground-truth adapter exposing only visible survivor positions."""

    max_range: int
    channel: str = "visual"
    base_detection_probability: float = 1.0
    smoke_attenuation: float = 1.0

    def __post_init__(self) -> None:
        if self.channel not in {"visual", "thermal"}:
            raise ValueError("survivor sensor channel must be visual or thermal")
        if self.max_range < 1:
            raise ValueError("survivor sensor range must be positive")
        if not 0.0 < self.base_detection_probability <= 1.0:
            raise ValueError("base detection probability must be in (0, 1]")
        if not 0.0 <= self.smoke_attenuation <= 1.0:
            raise ValueError("smoke attenuation must be in [0, 1]")

    def observe(self, world: GridWorld, origin: Position) -> tuple[Position, ...]:
        return self.observe_report(
            world,
            origin,
            smoke_profile="off",
            seed=0,
            step=0,
            observer_id="sensor",
        ).visible_survivors

    def can_observe(self, world: GridWorld, origin: Position, target: Position) -> bool:
        distance_squared = (target.x - origin.x) ** 2 + (target.y - origin.y) ** 2
        return (
            world.is_free(target)
            and distance_squared <= self.max_range**2
            and not any(
                not world.is_free(cell)
                for cell in _line_cells(origin, target)[1:-1]
            )
        )

    def observe_report(
        self,
        world: GridWorld,
        origin: Position,
        *,
        smoke_profile: str,
        seed: int,
        step: int,
        observer_id: str,
        perception_noise: str = "off",
    ) -> "SurvivorObservationReport":
        if not world.is_free(origin):
            raise ValueError("survivor sensor origin must be a free cell")

        if perception_noise not in PERCEPTION_NOISE_PROFILES:
            raise ValueError("unknown perception noise profile")
        observations: list[SurvivorObservation] = []
        attempts = 0
        degraded_attempts = 0
        successful_smoke_attempts = 0
        maximum_exposure = 0.0
        for survivor in sorted(world.survivors):
            distance_squared = (
                (survivor.x - origin.x) ** 2 + (survivor.y - origin.y) ** 2
            )
            if distance_squared > self.max_range**2:
                continue
            intervening_cells = _line_cells(origin, survivor)[1:-1]
            if any(not world.is_free(cell) for cell in intervening_cells):
                continue
            attempts += 1
            line = _line_cells(origin, survivor)
            exposure = (
                sum(world.smoke.density_at(cell) for cell in line) / len(line)
                if smoke_profile != "off"
                else 0.0
            )
            maximum_exposure = max(maximum_exposure, exposure)
            smoke_factor = max(
                0.05, 1.0 - self.smoke_attenuation * exposure
            )
            confidence = self.base_detection_probability * smoke_factor
            effective_range = (
                int(self.max_range * smoke_factor)
                if self.channel == "visual"
                else self.max_range
            )
            decision_score: float | None = None
            failure_reason: str | None = None
            smoke_degraded = False
            if distance_squared > effective_range**2:
                success = False
                failure_reason = "smoke_reduced_range"
                smoke_degraded = exposure > 0.0
            elif confidence < 1.0:
                identity_parts = (
                    (
                        str(seed),
                        smoke_profile,
                        observer_id,
                        str(step),
                        f"{origin.x},{origin.y}",
                        f"{survivor.x},{survivor.y}",
                    )
                    if self.channel == "visual"
                    else (
                        str(seed),
                        self.channel,
                        observer_id,
                        str(step),
                        f"{origin.x},{origin.y}",
                        f"{survivor.x},{survivor.y}",
                    )
                )
                identity = "|".join(identity_parts)
                decision_score = int.from_bytes(
                    sha256(identity.encode("utf-8")).digest()[:8], "big"
                ) / 2**64
                success = decision_score < confidence
                if not success:
                    smoke_degraded = (
                        exposure > 0.0
                        and decision_score < self.base_detection_probability
                    )
                    failure_reason = (
                        "smoke_confidence_threshold"
                        if smoke_degraded
                        else "confidence_threshold"
                    )
            else:
                success = True
            if not success:
                if smoke_degraded:
                    degraded_attempts += 1
            elif exposure > 0.0:
                successful_smoke_attempts += 1
            raw_success = success
            noise_score: float | None = None
            if raw_success and perception_noise != "off":
                noise_score = deterministic_unit(
                    "false-negative",
                    seed,
                    perception_noise,
                    self.channel,
                    observer_id,
                    step,
                    origin.x,
                    origin.y,
                    survivor.x,
                    survivor.y,
                )
                success = noise_score >= PERCEPTION_NOISE_PROFILES[
                    perception_noise
                ].false_negative_rate(self.channel)
                if not success:
                    failure_reason = "perception_false_negative"
                else:
                    distance = sqrt(distance_squared)
                    distance_factor = 0.75 + 0.25 * (
                        1.0 - min(1.0, distance / self.max_range)
                    )
                    confidence = (
                        confidence / self.base_detection_probability
                    ) * distance_factor * (
                        1.0
                        - PERCEPTION_NOISE_PROFILES[
                            perception_noise
                        ].false_negative_rate(self.channel)
                    )
            observations.append(
                SurvivorObservation(
                    position=survivor,
                    distance=sqrt(distance_squared),
                    smoke_exposure=exposure,
                    confidence=confidence,
                    decision_score=decision_score,
                    success=success,
                    smoke_degraded=smoke_degraded,
                    failure_reason=failure_reason,
                    raw_success=raw_success,
                    is_false_positive=False,
                    noise_score=noise_score,
                )
            )
        true_positives = sum(
            observation.success and not observation.is_false_positive
            for observation in observations
        )
        false_negatives = sum(
            observation.raw_success
            and not observation.success
            and not observation.is_false_positive
            for observation in observations
        )
        true_negatives = 0
        false_positives = 0
        if perception_noise != "off":
            candidates = []
            for y in range(world.height):
                for x in range(world.width):
                    candidate = Position(x, y)
                    distance_squared = (x - origin.x) ** 2 + (y - origin.y) ** 2
                    if (
                        candidate in world.survivors
                        or not world.is_free(candidate)
                        or distance_squared > self.max_range**2
                        or any(
                            not world.is_free(cell)
                            for cell in _line_cells(origin, candidate)[1:-1]
                        )
                    ):
                        continue
                    candidates.append(candidate)
            if candidates:
                candidate = min(
                    candidates,
                    key=lambda position: deterministic_unit(
                        "background-cell",
                        seed,
                        perception_noise,
                        self.channel,
                        observer_id,
                        step,
                        origin.x,
                        origin.y,
                        position.x,
                        position.y,
                    ),
                )
                score = deterministic_unit(
                    "false-positive",
                    seed,
                    perception_noise,
                    self.channel,
                    observer_id,
                    step,
                    origin.x,
                    origin.y,
                )
                profile = PERCEPTION_NOISE_PROFILES[perception_noise]
                if score < profile.false_positive_rate(self.channel):
                    confidence_unit = deterministic_unit(
                        "false-positive-confidence",
                        seed,
                        self.channel,
                        observer_id,
                        step,
                        candidate.x,
                        candidate.y,
                    )
                    confidence = profile.false_positive_confidence_min + (
                        profile.false_positive_confidence_max
                        - profile.false_positive_confidence_min
                    ) * confidence_unit
                    observations.append(
                        SurvivorObservation(
                            position=candidate,
                            distance=sqrt(
                                (candidate.x - origin.x) ** 2
                                + (candidate.y - origin.y) ** 2
                            ),
                            smoke_exposure=0.0,
                            confidence=confidence,
                            decision_score=score,
                            success=True,
                            smoke_degraded=False,
                            failure_reason=None,
                            raw_success=False,
                            is_false_positive=True,
                            noise_score=score,
                        )
                    )
                    false_positives = 1
                else:
                    true_negatives = 1
        visible = tuple(
            observation.position
            for observation in observations
            if observation.success
        )
        total_attempts = attempts + true_negatives + false_positives
        return SurvivorObservationReport(
            visible_survivors=visible,
            observations=tuple(observations),
            sensor_channel=self.channel,
            attempts=total_attempts,
            successful_observations=len(visible),
            failed_observations=total_attempts - len(visible),
            degraded_attempts=degraded_attempts,
            successful_smoke_observations=successful_smoke_attempts,
            maximum_smoke_exposure=maximum_exposure,
            true_positives=true_positives,
            false_positives=false_positives,
            true_negatives=true_negatives,
            false_negatives=false_negatives,
        )


@dataclass(frozen=True, slots=True)
class SurvivorObservationReport:
    visible_survivors: tuple[Position, ...]
    observations: tuple["SurvivorObservation", ...]
    sensor_channel: str
    attempts: int
    successful_observations: int
    failed_observations: int
    degraded_attempts: int
    successful_smoke_observations: int
    maximum_smoke_exposure: float
    true_positives: int = 0
    false_positives: int = 0
    true_negatives: int = 0
    false_negatives: int = 0


@dataclass(frozen=True, slots=True)
class SurvivorObservation:
    position: Position
    distance: float
    smoke_exposure: float
    confidence: float
    decision_score: float | None
    success: bool
    smoke_degraded: bool
    failure_reason: str | None
    raw_success: bool = True
    is_false_positive: bool = False
    noise_score: float | None = None
