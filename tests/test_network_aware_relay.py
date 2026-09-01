import unittest
from unittest.mock import patch

from echorescue.communication import CommunicationLink, CommunicationSnapshot, DroneConnection
from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.models import CellState, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.network_aware_relay import RelayUtilityInput, evaluate_relay_utility
from echorescue.network_aware_benchmark import _run_once
from echorescue.network_transport import DeterministicNetworkTransport, MessageType
from echorescue.replay import NETWORK_AWARE_REPLAY_SCHEMA_VERSION, generate_replay


def aware_config(**overrides: object) -> SimulationConfig:
    values = {
        "width": 9,
        "height": 7,
        "seed": 7,
        "drone_count": 2,
        "obstacle_density": 0.0,
        "communication_range": 3,
        "knowledge_mode": "local",
        "network_profile": "constrained",
        "relay_strategy": "network-aware",
        "network_relay_min_outage_steps": 1,
        "network_relay_utility_threshold": 0.0,
    }
    values.update(overrides)
    return SimulationConfig(**values)


def opportunity(*, survivor: bool = True) -> MultiDroneSimulation:
    simulation = MultiDroneSimulation(aware_config())
    relay = simulation.runtimes["drone-1"]
    scout = simulation.runtimes["drone-2"]
    relay.drone.position = Position(5, 1)
    scout.drone.position = Position(7, 1)
    known = {Position(x, 1): CellState.FREE for x in range(1, 8)}
    for runtime in (relay, scout):
        runtime.local_map.observe(known, step=1, source_id=runtime.drone.identifier)
        runtime.base_acknowledged_records = dict(runtime.local_map.records)
    if survivor:
        position = Position(7, 2)
        scout.confirmed_survivors.add(position)
        scout.detected_survivors.add(position)
    simulation.steps = 2
    simulation._sample_communication(record_events=False)
    simulation._current_outage_steps = {"drone-1": 2, "drone-2": 2}
    assert simulation.network_transport is not None
    simulation.network_transport._queued.clear()
    simulation.network_transport._in_flight.clear()
    return simulation


def utility(**overrides: object) -> RelayUtilityInput:
    values = {
        "confirmed_survivors": 1,
        "critical_status_units": 0,
        "recent_map_cells": 0,
        "general_map_cells": 0,
        "queue_units": 0,
        "route_hops": 2,
        "link_capacity_units": 36,
        "observed_delivery_ratio": 0.95,
        "ttl_remaining_steps": 128,
        "relay_travel_steps": 2,
        "energy_headroom": 50.0,
        "expected_direct_reconnect_steps": 20,
        "maximum_hops": 2,
        "utility_threshold": 25.0,
        "maximum_low_priority_backlog_units": 144,
    }
    values.update(overrides)
    return RelayUtilityInput(**values)


class NetworkAwareRelayTests(unittest.TestCase):
    def test_network_aware_mode_is_opt_in_and_constrained_only(self) -> None:
        self.assertEqual(SimulationConfig().relay_strategy, "off")
        with self.assertRaisesRegex(ValueError, "network_profile=constrained"):
            SimulationConfig(
                drone_count=2, knowledge_mode="local", relay_strategy="network-aware"
            )

    def test_no_deployment_without_valuable_data(self) -> None:
        simulation = opportunity(survivor=False)
        simulation._assign_network_aware_relay()
        self.assertEqual(simulation.relay_deployments, 0)

    def test_critical_survivor_payload_triggers_relay(self) -> None:
        simulation = opportunity()
        simulation._assign_network_aware_relay()
        self.assertEqual(simulation.relay_deployments, 1)
        self.assertTrue(any(
            event.event_type is EventType.NETWORK_RELAY_ACCEPTED
            for event in simulation.mission_log.events
        ))

    def test_energy_reserve_prevents_role_assignment(self) -> None:
        simulation = opportunity()
        for runtime in simulation.runtimes.values():
            runtime.battery.remaining = 10.0
        simulation._assign_network_aware_relay()
        self.assertEqual(simulation.relay_deployments, 0)

    def test_utility_accounts_for_ttl_capacity_and_delivery_history(self) -> None:
        good = evaluate_relay_utility(utility())
        poor = evaluate_relay_utility(utility(
            observed_delivery_ratio=0.05,
            ttl_remaining_steps=2,
            link_capacity_units=1,
        ))
        self.assertTrue(good.accepted)
        self.assertFalse(poor.accepted)
        self.assertEqual(poor.reason, "ttl_or_capacity_insufficient")

    def test_assignment_does_not_query_ground_truth(self) -> None:
        simulation = opportunity()
        with patch.object(GridWorld, "is_free", side_effect=AssertionError("GT leak")):
            simulation._assign_network_aware_relay()
        self.assertEqual(simulation.relay_deployments, 1)

    def test_decision_events_are_transition_deduplicated(self) -> None:
        simulation = opportunity()
        relay = simulation.runtimes["drone-1"]
        scout = simulation.runtimes["drone-2"]
        cells, survivors = simulation._relay_payload(scout)
        plan = simulation._relay_plan_for(relay, scout.drone.position)
        assert plan is not None
        decision = simulation._network_relay_decision(relay, scout, plan, cells, survivors)
        simulation._record_network_relay_decision(relay, decision, "drone-2")
        simulation._record_network_relay_decision(relay, decision, "drone-2")
        accepted = [e for e in simulation.mission_log.events if e.event_type is EventType.NETWORK_RELAY_ACCEPTED]
        self.assertEqual(len(accepted), 1)

    def test_backpressure_compacts_only_unsent_map_messages(self) -> None:
        network = DeterministicNetworkTransport(
            seed=1, profile="constrained", latency_steps=1,
            packet_loss_rate=0.0, link_capacity_units=2,
            max_fragment_units=2, fairness_age_steps=2,
        )
        network.enqueue(
            sender="drone-1", recipient="base", route=("drone-1", "base"),
            message_type=MessageType.MAP_UPDATE, payload=((Position(2, 2), "old"),),
            created_step=0, ttl=20,
        )
        compacted = network.compact_unsent_map_messages(sender="drone-1", recipient="base")
        self.assertEqual(len(compacted), 1)
        self.assertEqual(network.queue_size, 0)

    def test_critical_messages_precede_map_updates_under_backlog(self) -> None:
        network = DeterministicNetworkTransport(
            seed=1, profile="constrained", latency_steps=1,
            packet_loss_rate=0.0, link_capacity_units=1,
            max_fragment_units=1, fairness_age_steps=8,
        )
        common = dict(sender="drone-1", recipient="base", route=("drone-1", "base"), created_step=0, ttl=20)
        network.enqueue(message_type=MessageType.MAP_UPDATE, payload=("map",), **common)
        network.enqueue(message_type=MessageType.SURVIVOR_CONFIRMATION, payload=("S",), **common)
        snapshot = CommunicationSnapshot(
            nodes={"base": Position(0, 0), "drone-1": Position(1, 0)},
            links=(CommunicationLink.between("base", "drone-1"),),
            connections={"drone-1": DroneConnection(True, True)},
        )
        network.transmit(step=0, snapshot=snapshot)
        self.assertIs(network._in_flight[0].message_type, MessageType.SURVIVOR_CONFIRMATION)

    def test_route_hop_limit_rejects_long_reroute(self) -> None:
        network = DeterministicNetworkTransport(
            seed=1, profile="constrained", latency_steps=1,
            packet_loss_rate=0.0, link_capacity_units=2,
            max_fragment_units=2, fairness_age_steps=2,
        )
        network.maximum_route_hops = 2
        network.enqueue(
            sender="a", recipient="d", route=("a", "d"),
            message_type=MessageType.TELEMETRY, payload=(1,), created_step=0, ttl=20,
        )
        snapshot = CommunicationSnapshot(
            nodes={name: Position(i, 0) for i, name in enumerate("abcd")},
            links=tuple(CommunicationLink.between(*edge) for edge in (("a", "b"), ("b", "c"), ("c", "d"))),
            connections={},
        )
        network.transmit(step=0, snapshot=snapshot)
        self.assertEqual(network._queued[0].route, ("a", "d"))

    def test_rerouting_does_not_duplicate_logical_delivery(self) -> None:
        network = DeterministicNetworkTransport(
            seed=1, profile="constrained", latency_steps=1,
            packet_loss_rate=0.0, link_capacity_units=2,
            max_fragment_units=2, fairness_age_steps=2,
        )
        network.enqueue(
            sender="drone-2", recipient="base", route=("drone-2", "drone-1", "base"),
            message_type=MessageType.SURVIVOR_CONFIRMATION, payload=(Position(4, 4),),
            created_step=0, ttl=20,
        )
        radio = CommunicationSnapshot(
            nodes={"base": Position(0, 0), "drone-1": Position(1, 0), "drone-2": Position(2, 0)},
            links=(CommunicationLink.between("base", "drone-1"), CommunicationLink.between("drone-1", "drone-2")),
            connections={},
        )
        deliveries = []
        for step in range(4): deliveries.extend(network.advance(step=step, snapshot=radio))
        self.assertEqual(len(deliveries), 1)
        self.assertEqual(network.delivered_messages, 1)

    def test_scout_continues_exploration_after_relay_handoff(self) -> None:
        result = MultiDroneSimulation(SimulationConfig(
            seed=7, drone_count=2, communication_range=8, knowledge_mode="local",
            network_profile="constrained", relay_strategy="network-aware",
        )).run()
        metrics = result.network_aware_relay_metrics
        assert metrics is not None
        self.assertGreater(metrics["scout_exploration_steps_during_relay"], 0)

    def test_network_aware_mode_is_deterministic(self) -> None:
        config = SimulationConfig(
            seed=3, drone_count=2, communication_range=8, knowledge_mode="local",
            network_profile="constrained", relay_strategy="network-aware",
        )
        self.assertEqual(MultiDroneSimulation(config).run(), MultiDroneSimulation(config).run())

    def test_off_and_adaptive_seed_fingerprints_are_unchanged(self) -> None:
        common = dict(seed=7, drone_count=2, communication_range=8, knowledge_mode="local", network_profile="constrained")
        self.assertEqual(MultiDroneSimulation(SimulationConfig(**common, relay_strategy="off")).run().steps, 134)
        self.assertEqual(MultiDroneSimulation(SimulationConfig(**common, relay_strategy="adaptive")).run().steps, 142)

    def test_transport_only_ablation_has_compaction_without_relay_roles(self) -> None:
        first, first_audit = _run_once(7, "network-aware", False)
        second, second_audit = _run_once(7, "network-aware", False)
        metrics = first.network_aware_relay_metrics
        assert metrics is not None
        self.assertEqual(first.relay_deployments, 0)
        self.assertGreater(metrics["compacted_or_superseded_map_items"], 0)
        self.assertEqual(first.to_dict(), second.to_dict())
        self.assertEqual(first_audit, second_audit)
        self.assertTrue(first_audit["all_reachable_survivors_actively_confirmed"])

    def test_short_safety_suite_has_no_shield_interventions(self) -> None:
        for seed in range(5):
            result = MultiDroneSimulation(SimulationConfig(
                seed=seed, drone_count=2, communication_range=8, knowledge_mode="local",
                network_profile="constrained", relay_strategy="network-aware",
            )).run()
            self.assertTrue(result.mission_success)
            self.assertEqual(result.safety_shield_interventions, 0)

    def test_replay_exposes_versioned_inspectable_utility(self) -> None:
        replay = generate_replay(SimulationConfig(
            seed=7, drone_count=2, communication_range=8, knowledge_mode="local",
            network_profile="constrained", relay_strategy="network-aware",
        ))
        self.assertEqual(replay["schema_version"], NETWORK_AWARE_REPLAY_SCHEMA_VERSION)
        self.assertEqual(replay["mission"]["relay_strategy"], "network-aware")
        self.assertIn("network_aware_relay", replay["metrics"])
        active = [d["relay"] for f in replay["frames"] for d in f["drones"].values() if d["relay"]["active"]]
        self.assertTrue(active)
        self.assertTrue(all("utility" in relay for relay in active))


if __name__ == "__main__":
    unittest.main()
