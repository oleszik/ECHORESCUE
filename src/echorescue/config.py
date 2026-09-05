from dataclasses import dataclass

from echorescue.dynamic_obstacles import DYNAMIC_OBSTACLE_PROFILES
from echorescue.perception import PERCEPTION_NOISE_PROFILES
from echorescue.probabilistic import ProbabilityConfig, UNCERTAINTY_PROFILES
from echorescue.roles import ROLE_POLICIES
from echorescue.smoke import SMOKE_PROFILES

MIN_DRONE_COUNT = 1
MAX_DRONE_COUNT = 8


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """Central configuration for a deterministic Phase 1 mission."""

    width: int = 21
    height: int = 13
    seed: int = 7
    obstacle_density: float = 0.08
    sensor_range: int = 4
    survivor_count: int = 3
    survivor_sensor_range: int = 3
    survivor_sensor: str = "visual"
    thermal_survivor_sensor_range: int = 3
    thermal_detection_probability: float = 0.6
    thermal_smoke_attenuation: float = 0.15
    survivor_confirmation_observations: int = 2
    perception_noise: str = "off"
    uncertainty_profile: str = "off"
    planning_variant: str = "naive"
    probability_config: ProbabilityConfig = ProbabilityConfig()
    survivor_confirmation_evidence_threshold: float = 0.65
    survivor_rejection_evidence_threshold: float = 0.12
    survivor_negative_evidence_weight: float = 0.55
    battery_capacity: float = 220.0
    movement_energy_cost: float = 1.0
    sensor_energy_cost: float = 0.05
    energy_safety_reserve: float = 20.0
    drone_count: int = 1
    drone_start_positions: tuple[tuple[int, int], ...] | None = None
    wait_energy_cost: float = 0.05
    communication_range: int = 8
    proximity_sensor_range: int = 2
    intent_reservation_steps: int = 3
    motion_intent_ttl: int = 4
    deadlock_wait_threshold: int = 3
    distributed_deconfliction_enabled: bool = True
    relay_strategy: str = "off"
    relay_min_outage_steps: int = 40
    relay_min_unsynced_cells: int = 240
    relay_max_role_steps: int = 16
    relay_cooldown_steps: int = 8
    relay_max_deployments: int = 1
    relay_energy_margin: float = 5.0
    relay_min_benefit_ratio: float = 20.0
    network_relay_utility_threshold: float = 25.0
    network_relay_max_backlog_units: int = 144
    network_relay_max_hops: int = 2
    network_relay_hysteresis_steps: int = 3
    network_relay_recent_map_age_steps: int = 8
    network_relay_map_delta_limit: int = 24
    network_relay_min_outage_steps: int = 8
    multi_relay_max_active: int = 2
    multi_relay_activation_outage_steps: int = 3
    multi_relay_min_unsynced_cells: int = 12
    multi_relay_min_hold_steps: int = 6
    multi_relay_max_role_steps: int = 24
    multi_relay_deactivation_hysteresis_steps: int = 3
    multi_relay_max_deployments: int = 8
    multi_relay_candidate_limit: int = 48
    relay_prediction_horizon: int = 4
    network_profile: str = "ideal"
    network_latency_steps: int = 1
    network_packet_loss_rate: float = 0.05
    network_link_capacity_units: int = 36
    network_max_fragment_units: int = 12
    network_map_ttl: int = 256
    network_survivor_ttl: int = 128
    network_fairness_age_steps: int = 8
    network_backlog_warning_threshold: int = 24
    final_sync_max_steps: int = 128
    knowledge_mode: str = "shared"
    local_map_shadow_mode: bool | None = None
    base_knowledge_store_enabled: bool = True
    failure_schedule: tuple[tuple[str, int], ...] = ()
    role_policy: str = "off"
    dynamic_obstacles: str = "off"
    dynamic_obstacle_schedule: tuple[tuple[int, int, int], ...] = ()
    smoke_profile: str = "off"
    max_steps: int = 1_000

    def __post_init__(self) -> None:
        if self.uncertainty_profile not in {"off", *UNCERTAINTY_PROFILES}:
            raise ValueError("unknown uncertainty profile")
        if self.planning_variant not in {"naive", "uncertainty-aware"}:
            raise ValueError("unknown planning variant")
        if self.uncertainty_profile != "off":
            if self.perception_noise not in {"off", self.uncertainty_profile}:
                raise ValueError("uncertainty profile replaces legacy noise configuration")
            object.__setattr__(self, "perception_noise", self.uncertainty_profile)
        elif self.planning_variant != "naive":
            raise ValueError("uncertainty-aware planning requires probabilistic perception")
        if self.width < 7 or self.height < 7:
            raise ValueError("width and height must both be at least 7")
        if not 0.0 <= self.obstacle_density <= 0.35:
            raise ValueError("obstacle_density must be between 0.0 and 0.35")
        if self.sensor_range < 1:
            raise ValueError("sensor_range must be positive")
        interior_cells = (self.width - 2) * (self.height - 2)
        if not 0 <= self.survivor_count < interior_cells:
            raise ValueError("survivor_count must fit within the interior grid")
        if self.survivor_sensor_range < 1:
            raise ValueError("survivor_sensor_range must be positive")
        if self.survivor_sensor not in {"visual", "thermal"}:
            raise ValueError("survivor_sensor must be visual or thermal")
        if self.thermal_survivor_sensor_range < 1:
            raise ValueError("thermal_survivor_sensor_range must be positive")
        if not 0.0 < self.thermal_detection_probability <= 1.0:
            raise ValueError(
                "thermal_detection_probability must be in (0, 1]"
            )
        if not 0.0 <= self.thermal_smoke_attenuation <= 1.0:
            raise ValueError("thermal_smoke_attenuation must be in [0, 1]")
        if self.survivor_confirmation_observations < 2:
            raise ValueError("survivor confirmation requires at least two observations")
        if self.perception_noise not in PERCEPTION_NOISE_PROFILES:
            raise ValueError(
                "perception_noise must be one of "
                + ", ".join(sorted(PERCEPTION_NOISE_PROFILES))
            )
        if not 0.0 < self.survivor_confirmation_evidence_threshold <= 1.0:
            raise ValueError("survivor confirmation evidence must be in (0, 1]")
        if not 0.0 <= self.survivor_rejection_evidence_threshold < 1.0:
            raise ValueError("survivor rejection evidence must be in [0, 1)")
        if not 0.0 < self.survivor_negative_evidence_weight <= 1.0:
            raise ValueError("survivor negative evidence weight must be in (0, 1]")
        if self.battery_capacity <= 0:
            raise ValueError("battery_capacity must be positive")
        if self.movement_energy_cost <= 0:
            raise ValueError("movement_energy_cost must be positive")
        if self.sensor_energy_cost < 0:
            raise ValueError("sensor_energy_cost must not be negative")
        if not 0 <= self.energy_safety_reserve < self.battery_capacity:
            raise ValueError("energy_safety_reserve must be below battery_capacity")
        if not MIN_DRONE_COUNT <= self.drone_count <= MAX_DRONE_COUNT:
            raise ValueError(
                f"drone_count must be between {MIN_DRONE_COUNT} and "
                f"{MAX_DRONE_COUNT}"
            )
        if self.drone_start_positions is not None:
            if len(self.drone_start_positions) != self.drone_count:
                raise ValueError("one start position is required per drone")
            if any(
                not (0 < x < self.width - 1 and 0 < y < self.height - 1)
                for x, y in self.drone_start_positions
            ):
                raise ValueError("drone start positions must be interior cells")
        if self.wait_energy_cost < 0:
            raise ValueError("wait_energy_cost must not be negative")
        if self.communication_range < 1:
            raise ValueError("communication_range must be positive")
        if self.proximity_sensor_range < 1:
            raise ValueError("proximity_sensor_range must be positive")
        if not 2 <= self.intent_reservation_steps <= 3:
            raise ValueError("intent_reservation_steps must be 2 or 3")
        if self.motion_intent_ttl < 1:
            raise ValueError("motion_intent_ttl must be positive")
        if self.deadlock_wait_threshold < 2:
            raise ValueError("deadlock_wait_threshold must be at least 2")
        if self.relay_strategy not in {
            "off",
            "adaptive",
            "network-aware",
            "multi-relay",
            "predictive",
        }:
            raise ValueError(
                "relay_strategy must be off, adaptive, network-aware, "
                "multi-relay, or predictive"
            )
        if self.relay_min_outage_steps < 1:
            raise ValueError("relay_min_outage_steps must be positive")
        if self.relay_min_unsynced_cells < 1:
            raise ValueError("relay_min_unsynced_cells must be positive")
        if self.relay_max_role_steps < 1:
            raise ValueError("relay_max_role_steps must be positive")
        if self.relay_cooldown_steps < 1:
            raise ValueError("relay_cooldown_steps must be positive")
        if self.relay_max_deployments < 1:
            raise ValueError("relay_max_deployments must be positive")
        if self.relay_energy_margin < 0:
            raise ValueError("relay_energy_margin must not be negative")
        if self.relay_min_benefit_ratio <= 0:
            raise ValueError("relay_min_benefit_ratio must be positive")
        if self.network_relay_utility_threshold < 0:
            raise ValueError("network_relay_utility_threshold must not be negative")
        if self.network_relay_max_backlog_units < 1:
            raise ValueError("network_relay_max_backlog_units must be positive")
        if self.network_relay_max_hops < 1:
            raise ValueError("network_relay_max_hops must be positive")
        if self.network_relay_hysteresis_steps < 1:
            raise ValueError("network_relay_hysteresis_steps must be positive")
        if self.network_relay_recent_map_age_steps < 1:
            raise ValueError("network_relay_recent_map_age_steps must be positive")
        if self.network_relay_map_delta_limit < 1:
            raise ValueError("network_relay_map_delta_limit must be positive")
        if self.network_relay_min_outage_steps < 1:
            raise ValueError("network_relay_min_outage_steps must be positive")
        if not 1 <= self.multi_relay_max_active <= 2:
            raise ValueError("multi_relay_max_active must be 1 or 2")
        if self.multi_relay_activation_outage_steps < 1:
            raise ValueError(
                "multi_relay_activation_outage_steps must be positive"
            )
        if self.multi_relay_min_unsynced_cells < 1:
            raise ValueError("multi_relay_min_unsynced_cells must be positive")
        if self.multi_relay_min_hold_steps < 1:
            raise ValueError("multi_relay_min_hold_steps must be positive")
        if self.multi_relay_max_role_steps < self.multi_relay_min_hold_steps:
            raise ValueError(
                "multi_relay_max_role_steps must cover minimum hold time"
            )
        if self.multi_relay_deactivation_hysteresis_steps < 1:
            raise ValueError(
                "multi_relay_deactivation_hysteresis_steps must be positive"
            )
        if self.multi_relay_max_deployments < 1:
            raise ValueError("multi_relay_max_deployments must be positive")
        if self.multi_relay_candidate_limit < 4:
            raise ValueError("multi_relay_candidate_limit must be at least 4")
        if self.relay_prediction_horizon < 1:
            raise ValueError("relay_prediction_horizon must be positive")
        if self.network_profile not in {"ideal", "constrained"}:
            raise ValueError("network_profile must be ideal or constrained")
        if self.network_profile == "constrained" and self.effective_knowledge_mode != "local":
            raise ValueError("constrained network requires knowledge_mode=local")
        if self.network_latency_steps < 1:
            raise ValueError("network_latency_steps must be positive")
        if not 0.0 <= self.network_packet_loss_rate < 1.0:
            raise ValueError("network_packet_loss_rate must be in [0, 1)")
        if self.network_link_capacity_units < 1:
            raise ValueError("network_link_capacity_units must be positive")
        if not 1 <= self.network_max_fragment_units <= self.network_link_capacity_units:
            raise ValueError("network_max_fragment_units must fit the link capacity")
        if self.network_map_ttl < 1 or self.network_survivor_ttl < 1:
            raise ValueError("network knowledge TTLs must be positive")
        if self.network_fairness_age_steps < 1:
            raise ValueError("network_fairness_age_steps must be positive")
        if self.network_backlog_warning_threshold < 1:
            raise ValueError("network_backlog_warning_threshold must be positive")
        if self.final_sync_max_steps < 1:
            raise ValueError("final_sync_max_steps must be positive")
        if self.knowledge_mode not in {"shared", "shadow", "local"}:
            raise ValueError("knowledge_mode must be shared, shadow, or local")
        valid_drone_ids = {
            f"drone-{index}" for index in range(1, self.drone_count + 1)
        }
        seen_failure_ids: set[str] = set()
        for drone_id, step in self.failure_schedule:
            if drone_id not in valid_drone_ids:
                raise ValueError("failure drone ID must belong to the configured fleet")
            if drone_id in seen_failure_ids:
                raise ValueError("each drone may have only one injected failure")
            if step < 0 or step >= self.max_steps:
                raise ValueError("failure step must be within the mission step limit")
            seen_failure_ids.add(drone_id)
        if self.role_policy not in ROLE_POLICIES:
            raise ValueError(
                "role_policy must be one of "
                + ", ".join(sorted(ROLE_POLICIES))
            )
        if self.dynamic_obstacles not in DYNAMIC_OBSTACLE_PROFILES:
            raise ValueError(
                "dynamic_obstacles must be one of "
                + ", ".join(sorted(DYNAMIC_OBSTACLE_PROFILES))
            )
        if self.dynamic_obstacles != "off" and self.dynamic_obstacle_schedule:
            raise ValueError(
                "dynamic obstacle profile and explicit injections are exclusive"
            )
        seen_obstacles: set[tuple[int, int]] = set()
        configured_starts = self.drone_start_positions or ()
        for x, y, step in self.dynamic_obstacle_schedule:
            if not (0 < x < self.width - 1 and 0 < y < self.height - 1):
                raise ValueError("dynamic obstacle must be an interior cell")
            if (x, y) == (1, 1):
                raise ValueError("dynamic obstacle cannot block the base")
            if (x, y) in configured_starts:
                raise ValueError("dynamic obstacle cannot block an agent start")
            if (x, y) in seen_obstacles:
                raise ValueError("a dynamic obstacle location may occur only once")
            if step < 1 or step >= self.max_steps:
                raise ValueError("dynamic obstacle step must be within mission steps")
            seen_obstacles.add((x, y))
        if self.smoke_profile not in SMOKE_PROFILES:
            raise ValueError(
                "smoke_profile must be one of "
                + ", ".join(sorted(SMOKE_PROFILES))
            )
        if (
            self.relay_strategy
            in {"adaptive", "network-aware", "multi-relay", "predictive"}
            and self.effective_knowledge_mode != "local"
        ):
            raise ValueError("Relay strategies require knowledge_mode=local")
        if (
            self.relay_strategy in {"adaptive", "network-aware"}
            and self.drone_count != 2
        ):
            raise ValueError(
                "adaptive relay strategies currently require exactly two drones"
            )
        if (
            self.relay_strategy
            in {"network-aware", "multi-relay", "predictive"}
            and self.network_profile != "constrained"
        ):
            raise ValueError(
                "network-aware and N-agent Relay require "
                "network_profile=constrained"
            )
        if (
            self.relay_strategy in {"multi-relay", "predictive"}
            and self.multi_relay_max_active >= self.drone_count
        ):
            raise ValueError(
                "multi_relay_max_active must leave at least one non-Relay agent"
            )
        if self.max_steps < 1:
            raise ValueError("max_steps must be positive")

    @property
    def effective_knowledge_mode(self) -> str:
        if self.local_map_shadow_mode is None:
            return self.knowledge_mode
        return "shadow" if self.local_map_shadow_mode else "shared"

    @property
    def active_survivor_sensor_range(self) -> int:
        if self.survivor_sensor == "thermal":
            return self.thermal_survivor_sensor_range
        return self.survivor_sensor_range

    @property
    def active_survivor_detection_probability(self) -> float:
        if self.survivor_sensor == "thermal":
            return self.thermal_detection_probability
        return 1.0

    @property
    def active_survivor_smoke_attenuation(self) -> float:
        if self.survivor_sensor == "thermal":
            return self.thermal_smoke_attenuation
        return 1.0
