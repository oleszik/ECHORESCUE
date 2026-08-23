import json
import unittest
from dataclasses import replace
from pathlib import Path

from echorescue.communication import (
    CommunicationLink,
    CommunicationSnapshot,
    DroneConnection,
)
from echorescue.cli import build_parser
from echorescue.models import Position
from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.knowledge import CellKnowledge, KnowledgeMap
from echorescue.models import CellState, DroneStatus
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.network_benchmark import run_network_benchmark
from echorescue.network_transport import (
    DeterministicNetworkTransport,
    MessageType,
)
from echorescue.replay import generate_replay, replay_json_bytes


def snapshot(*links: tuple[str, str]) -> CommunicationSnapshot:
    nodes = {
        "base": Position(0, 0),
        "drone-1": Position(1, 0),
        "drone-2": Position(2, 0),
    }
    return CommunicationSnapshot(
        nodes=nodes,
        links=tuple(sorted(CommunicationLink.between(*link) for link in links)),
        connections={
            "drone-1": DroneConnection(True, True),
            "drone-2": DroneConnection(True, False, ("base", "drone-1", "drone-2")),
        },
    )


def transport(**overrides: object) -> DeterministicNetworkTransport:
    options = {
        "seed": 7,
        "profile": "constrained",
        "latency_steps": 1,
        "packet_loss_rate": 0.0,
        "link_capacity_units": 2,
        "max_fragment_units": 2,
        "fairness_age_steps": 1,
    }
    options.update(overrides)
    return DeterministicNetworkTransport(**options)


class NetworkTransportTests(unittest.TestCase):
    def test_cli_exposes_central_transport_parameters(self) -> None:
        args = build_parser().parse_args(
            [
                "--knowledge-mode", "local",
                "--network-profile", "constrained",
                "--network-latency-steps", "2",
                "--network-packet-loss", "0.1",
                "--network-link-capacity", "9",
                "--network-fragment-size", "3",
                "--final-sync-max-steps", "17",
            ]
        )
        self.assertEqual(args.network_profile, "constrained")
        self.assertEqual(args.network_latency_steps, 2)
        self.assertEqual(args.network_packet_loss, 0.1)
        self.assertEqual(args.network_link_capacity, 9)
        self.assertEqual(args.network_fragment_size, 3)
        self.assertEqual(args.final_sync_max_steps, 17)

    def test_constrained_profile_requires_active_local_knowledge(self) -> None:
        with self.assertRaisesRegex(ValueError, "knowledge_mode=local"):
            SimulationConfig(
                drone_count=2,
                knowledge_mode="shared",
                network_profile="constrained",
            )

    def test_delivery_only_becomes_effective_after_latency(self) -> None:
        network = transport()
        network.enqueue(
            sender="drone-1",
            recipient="base",
            route=("drone-1", "base"),
            message_type=MessageType.SURVIVOR_CONFIRMATION,
            payload=(Position(4, 5),),
            created_step=0,
            ttl=10,
        )
        self.assertEqual(network.advance(step=0, snapshot=snapshot(("drone-1", "base"))), ())
        delivered = network.advance(step=1, snapshot=snapshot(("drone-1", "base")))
        self.assertEqual(delivered[0].payload, (Position(4, 5),))
        self.assertEqual(delivered[0].latency, 1)

    def test_bandwidth_fragments_large_map_update_across_steps(self) -> None:
        network = transport()
        network.enqueue(
            sender="drone-1",
            recipient="base",
            route=("drone-1", "base"),
            message_type=MessageType.MAP_UPDATE,
            payload=tuple(range(5)),
            created_step=0,
            ttl=20,
        )
        delivered = []
        radio = snapshot(("drone-1", "base"))
        for step in range(4):
            delivered.extend(network.advance(step=step, snapshot=radio))
        self.assertEqual([len(item.payload) for item in delivered], [2, 2, 1])
        self.assertEqual(network.delivered_messages, 1)

    def test_loss_is_seed_stable_and_enqueue_order_independent(self) -> None:
        def run(order: tuple[str, ...]) -> dict[str, str]:
            network = transport(packet_loss_rate=0.5, link_capacity_units=4)
            for name in order:
                network.enqueue(
                    sender="drone-1",
                    recipient="base",
                    route=("drone-1", "base"),
                    message_type=MessageType.TELEMETRY,
                    payload=(name,),
                    created_step=0,
                    ttl=3,
                    message_key=name,
                )
            network.advance(step=0, snapshot=snapshot(("drone-1", "base")))
            return {
                event.message_id: event.event_type
                for event in network.drain_events()
                if event.event_type == "message_lost"
            }

        self.assertEqual(run(("a", "b")), run(("b", "a")))

    def test_lost_fragment_has_no_delivery_side_effect(self) -> None:
        network = transport(packet_loss_rate=0.999999999999)
        network.enqueue(
            sender="drone-1",
            recipient="base",
            route=("drone-1", "base"),
            message_type=MessageType.SURVIVOR_CONFIRMATION,
            payload=(Position(3, 3),),
            created_step=0,
            ttl=1,
        )
        radio = snapshot(("drone-1", "base"))
        self.assertEqual(network.advance(step=0, snapshot=radio), ())
        self.assertEqual(network.advance(step=1, snapshot=radio), ())
        self.assertGreater(network.lost_fragments, 0)
        self.assertEqual(network.delivered_fragments, 0)

    def test_safety_messages_precede_map_updates(self) -> None:
        network = transport(link_capacity_units=1, max_fragment_units=1)
        common = {
            "sender": "drone-1",
            "recipient": "drone-2",
            "route": ("drone-1", "drone-2"),
            "created_step": 0,
            "ttl": 10,
        }
        network.enqueue(message_type=MessageType.MAP_UPDATE, payload=("map",), **common)
        network.enqueue(message_type=MessageType.MOTION_INTENT, payload=("intent",), **common)
        network.advance(step=0, snapshot=snapshot(("drone-1", "drone-2")))
        self.assertEqual(network._in_flight[0].message_type, MessageType.MOTION_INTENT)

    def test_aging_prevents_low_priority_starvation(self) -> None:
        network = transport(link_capacity_units=1, max_fragment_units=1)
        radio = snapshot(("drone-1", "drone-2"))
        network.enqueue(
            sender="drone-1",
            recipient="drone-2",
            route=("drone-1", "drone-2"),
            message_type=MessageType.TELEMETRY,
            payload=("low",),
            created_step=0,
            ttl=20,
        )
        deliveries = []
        for step in range(8):
            network.enqueue(
                sender="drone-1",
                recipient="drone-2",
                route=("drone-1", "drone-2"),
                message_type=MessageType.MOTION_INTENT,
                payload=(f"urgent-{step}",),
                created_step=step,
                ttl=20,
            )
            deliveries.extend(network.advance(step=step, snapshot=radio))
        self.assertTrue(any(item.payload == ("low",) for item in deliveries))

    def test_ttl_expires_stale_motion_intent(self) -> None:
        network = transport()
        network.enqueue(
            sender="drone-1",
            recipient="drone-2",
            route=("drone-1", "drone-2"),
            message_type=MessageType.MOTION_INTENT,
            payload=("intent",),
            created_step=0,
            ttl=1,
        )
        disconnected = snapshot()
        for step in range(3):
            network.advance(step=step, snapshot=disconnected)
        self.assertEqual(network.stale_intents, 1)
        self.assertEqual(network.queue_size, 0)

    def test_relay_requires_two_link_transmissions(self) -> None:
        network = transport()
        network.enqueue(
            sender="drone-2",
            recipient="base",
            route=("drone-2", "drone-1", "base"),
            message_type=MessageType.SURVIVOR_CONFIRMATION,
            payload=(Position(8, 8),),
            created_step=0,
            ttl=10,
        )
        radio = snapshot(("drone-2", "drone-1"), ("drone-1", "base"))
        self.assertEqual(network.advance(step=0, snapshot=radio), ())
        self.assertEqual(network.advance(step=1, snapshot=radio), ())
        delivered = network.advance(step=2, snapshot=radio)
        self.assertEqual(delivered[0].recipient, "base")
        self.assertTrue(delivered[0].relayed)
        self.assertEqual(network.relay_fragments_forwarded, 1)

    def test_invalid_queued_route_is_repaired_from_current_topology(self) -> None:
        network = transport()
        network.enqueue(
            sender="drone-2",
            recipient="base",
            route=("drone-2", "drone-1", "base"),
            message_type=MessageType.SURVIVOR_CONFIRMATION,
            payload=(Position(5, 5),),
            created_step=0,
            ttl=10,
        )
        direct = snapshot(("drone-2", "base"))
        self.assertEqual(network.advance(step=0, snapshot=direct), ())
        delivered = network.advance(step=1, snapshot=direct)
        self.assertEqual(delivered[0].route, ("drone-2", "base"))
        self.assertEqual(network.routes_replanned, 1)

    def test_older_map_observation_cannot_replace_newer_record(self) -> None:
        knowledge = KnowledgeMap(7, 7)
        position = Position(3, 3)
        newer = CellKnowledge(CellState.FREE, observed_step=9, source_id="drone-2")
        older = CellKnowledge(CellState.FREE, observed_step=4, source_id="drone-1")
        knowledge.apply(((position, newer),))
        self.assertEqual(knowledge.apply(((position, older),)), ())
        self.assertEqual(knowledge.record_at(position), newer)

    def test_delivery_ratio_denominators_are_distinct(self) -> None:
        network = transport(link_capacity_units=1, max_fragment_units=1)
        network.enqueue(
            sender="drone-1",
            recipient="base",
            route=("drone-1", "base"),
            message_type=MessageType.MAP_UPDATE,
            payload=("a", "b"),
            created_step=0,
            ttl=10,
        )
        radio = snapshot(("drone-1", "base"))
        network.advance(step=0, snapshot=radio)
        network.advance(step=1, snapshot=radio)
        network.finalize(step=1)
        self.assertEqual(network.transmission_attempts, 2)
        self.assertEqual(network.fragment_attempt_delivery_ratio, 1.0)
        self.assertEqual(network.created_fragments, 2)
        self.assertEqual(network.unique_fragment_eventual_delivery_ratio, 0.5)
        self.assertEqual(network.logical_message_completion_ratio, 0.0)
        self.assertEqual(network.dropped_fragments, 1)


class ConstrainedNetworkIntegrationTests(unittest.TestCase):
    def test_constrained_replay_is_versioned_deterministic_and_operator_safe(self) -> None:
        config = SimulationConfig(
            seed=7,
            drone_count=2,
            knowledge_mode="local",
            network_profile="constrained",
        )
        first = generate_replay(config)
        second = generate_replay(config)
        self.assertEqual(replay_json_bytes(first), replay_json_bytes(second))
        self.assertEqual(first["schema_version"], "1.7")
        self.assertEqual(first["mission"]["network_profile"], "constrained")
        self.assertTrue(all("network" in frame for frame in first["frames"]))
        payload = json.dumps(first, sort_keys=True)
        self.assertNotIn('"ground_truth"', payload)
        self.assertNotIn('"survivor_positions"', payload)

    def test_ideal_profile_retains_existing_replay_semantics(self) -> None:
        artifact = json.loads(
            (
                Path(__file__).resolve().parents[1]
                / "replays"
                / "seed_7_local.json"
            ).read_text(encoding="utf-8")
        )
        generated = generate_replay(
            SimulationConfig(seed=7, drone_count=2, knowledge_mode="local")
        )
        self.assertEqual(generated, artifact)
        self.assertNotIn("network_profile", generated["mission"])
        self.assertTrue(all("network" not in frame for frame in generated["frames"]))

    def test_proximity_remains_available_when_no_fresh_intent_arrives(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=9,
                height=7,
                seed=7,
                drone_count=2,
                obstacle_density=0.0,
                communication_range=1,
                knowledge_mode="local",
                network_profile="constrained",
            )
        )
        boundary = {
            Position(x, y)
            for y in range(7)
            for x in range(9)
            if x in (0, 8) or y in (0, 6)
        }
        simulation.world = GridWorld(
            9, 7, Position(1, 1), frozenset(boundary | {Position(2, 3)})
        )
        first, second, destination = Position(2, 2), Position(3, 3), Position(3, 2)
        simulation.runtimes["drone-1"].drone.position = first
        simulation.runtimes["drone-2"].drone.position = second
        for runtime in simulation.runtimes.values():
            runtime.local_map.observe(
                {first: CellState.FREE, second: CellState.FREE, destination: CellState.FREE},
                step=1,
                source_id=runtime.drone.identifier,
            )
        simulation.steps = 1
        simulation._sample_communication(record_events=False)
        resolved = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )
        self.assertEqual(sum(value == destination for value in resolved.values()), 1)
        self.assertEqual(simulation.proximity_avoidances_without_fresh_intent, 1)

    def test_small_three_profile_benchmark_repeats_each_seed(self) -> None:
        first = run_network_benchmark(1)
        second = run_network_benchmark(1)
        self.assertEqual(first, second)
        self.assertEqual(first["suite"]["runs_per_seed"], 2)
        self.assertEqual(first["suite"]["determinism_check"], "passed")

    def _land_with_local_confirmation(
        self, config: SimulationConfig
    ) -> tuple[MultiDroneSimulation, Position]:
        simulation = MultiDroneSimulation(config)
        survivor = next(iter(simulation.world.survivors))
        runtime = simulation.runtimes["drone-1"]
        runtime.confirmed_survivors.add(survivor)
        runtime.detected_survivors.add(survivor)
        for item in simulation.runtimes.values():
            item.drone.position = simulation.world.base
            item.drone.status = DroneStatus.LANDED
        simulation._sample_communication(record_events=False)
        simulation._update_completion()
        return simulation, survivor

    def test_landed_drone_delivers_survivor_during_real_final_sync(self) -> None:
        simulation, survivor = self._land_with_local_confirmation(
            SimulationConfig(
                seed=7,
                drone_count=2,
                knowledge_mode="local",
                network_profile="constrained",
                network_latency_steps=2,
                network_link_capacity_units=1,
                network_max_fragment_units=1,
                network_packet_loss_rate=0.0,
                final_sync_max_steps=10,
            )
        )
        self.assertTrue(simulation._final_sync_active)
        while simulation.step():
            pass
        result = simulation.result()
        self.assertIn(survivor, simulation._base_confirmed_survivors)
        self.assertTrue(result.final_sync_started)
        self.assertGreaterEqual(result.final_sync_duration, 2)
        self.assertFalse(result.final_sync_timeout)
        self.assertEqual(result.final_sync_survivor_confirmations_transferred, 1)
        self.assertLess(
            simulation.base_knowledge_map.known_cell_count,
            simulation.runtimes["drone-1"].local_map.known_cell_count,
        )

    def test_final_sync_respects_loss_and_times_out_without_magic_recall(self) -> None:
        simulation, _survivor = self._land_with_local_confirmation(
            SimulationConfig(
                seed=7,
                drone_count=2,
                knowledge_mode="local",
                network_profile="constrained",
                network_packet_loss_rate=0.999999999999,
                final_sync_max_steps=2,
            )
        )
        while simulation.step():
            pass
        result = simulation.result()
        self.assertTrue(result.final_sync_timeout)
        self.assertEqual(result.termination_reason, "final_sync_timeout")
        self.assertEqual(result.survivor_recall_at_base, 0.0)
        self.assertEqual(result.base_survivors_confirmed, 0)
        self.assertEqual(
            result.local_survivors_confirmed_by_drone["drone-1"], 1
        )
        event_types = {event.event_type for event in result.mission_events}
        self.assertIn(EventType.FINAL_SYNC_STARTED, event_types)
        self.assertIn(EventType.FINAL_SYNC_TIMEOUT, event_types)

    def test_final_sync_retransmits_deterministic_packet_loss(self) -> None:
        simulation, survivor = self._land_with_local_confirmation(
            SimulationConfig(
                seed=7,
                drone_count=2,
                knowledge_mode="local",
                network_profile="constrained",
                network_packet_loss_rate=0.5,
                final_sync_max_steps=30,
            )
        )
        while simulation.step():
            pass
        result = simulation.result()
        self.assertIn(survivor, simulation._base_confirmed_survivors)
        self.assertGreater(result.final_sync_retransmissions, 0)
        self.assertGreater(result.network_fragments_lost, 0)
        self.assertFalse(result.final_sync_timeout)

    def test_current_proximity_prevents_delayed_edge_swap_before_shield(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=9,
                height=7,
                seed=7,
                drone_count=2,
                obstacle_density=0.0,
                knowledge_mode="local",
                network_profile="constrained",
            )
        )
        first, second = Position(2, 2), Position(3, 2)
        simulation.runtimes["drone-1"].drone.position = first
        simulation.runtimes["drone-2"].drone.position = second
        for runtime in simulation.runtimes.values():
            runtime.local_map.observe(
                {first: CellState.FREE, second: CellState.FREE},
                step=1,
                source_id=runtime.drone.identifier,
            )
        simulation.steps = 3
        simulation._sample_communication(record_events=False)
        stale_first = simulation._motion_intent(
            simulation.runtimes["drone-1"], first
        )
        stale_second = simulation._motion_intent(
            simulation.runtimes["drone-2"], second
        )
        stale_second = replace(
            stale_second, current_position=Position(4, 2)
        )
        simulation._received_motion_intents["drone-2"]["drone-1"] = stale_first
        simulation._received_motion_intents["drone-1"]["drone-2"] = stale_second
        resolved = simulation._deconflict_intentions(
            {"drone-1": second, "drone-2": first}
        )
        simulation._execute_intentions(resolved)
        self.assertEqual(simulation.safety_shield_interventions, 0)
        self.assertEqual(
            simulation.runtimes["drone-1"].drone.position, first
        )
        self.assertEqual(
            simulation.runtimes["drone-2"].drone.position, second
        )

    def test_expired_peer_intent_falls_back_to_proximity_before_shield(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=9,
                height=7,
                seed=7,
                drone_count=2,
                obstacle_density=0.0,
                knowledge_mode="local",
                network_profile="constrained",
            )
        )
        first, second = Position(2, 2), Position(3, 2)
        simulation.runtimes["drone-1"].drone.position = first
        simulation.runtimes["drone-2"].drone.position = second
        for runtime in simulation.runtimes.values():
            runtime.local_map.observe(
                {first: CellState.FREE, second: CellState.FREE},
                step=1,
                source_id=runtime.drone.identifier,
            )
        simulation.steps = 10
        simulation._sample_communication(record_events=False)
        expired_first = replace(
            simulation._motion_intent(
                simulation.runtimes["drone-1"], second
            ),
            valid_until_step=9,
        )
        expired_second = replace(
            simulation._motion_intent(
                simulation.runtimes["drone-2"], first
            ),
            valid_until_step=9,
        )
        simulation._received_motion_intents["drone-2"]["drone-1"] = expired_first
        simulation._received_motion_intents["drone-1"]["drone-2"] = expired_second
        resolved = simulation._deconflict_intentions(
            {"drone-1": second, "drone-2": first}
        )
        simulation._execute_intentions(resolved)
        self.assertEqual(simulation.safety_shield_interventions, 0)
        self.assertEqual(simulation.proximity_avoidances_without_fresh_intent, 1)

    def test_constrained_corridor_clearance_avoids_starvation_and_shield(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=27,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
                network_profile="constrained",
            )
        ).run()
        self.assertTrue(result.mission_success)
        self.assertEqual(result.drones_returned, 2)
        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertEqual(result.safety_shield_interventions, 0)
        self.assertGreater(result.local_replans_due_to_drones, 0)


if __name__ == "__main__":
    unittest.main()
