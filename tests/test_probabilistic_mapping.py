import unittest
from dataclasses import replace

from echorescue.knowledge import ProbabilisticKnowledgeMap, merge_knowledge_maps
from echorescue.models import CellState, Position
from echorescue.probabilistic import ProbabilityConfig, OccupancyEvidence, evaluate
from echorescue.perception import SurvivorHypothesisTracker
from echorescue.sensors import uncertain_observations
from echorescue.multi_floor import MultiFloorKnowledge, GridPosition


class ProbabilisticMappingTests(unittest.TestCase):
    def test_prior_thresholds_bounds_and_conflict(self):
        config = ProbabilityConfig()
        self.assertEqual(evaluate((), 0, config).probability, .5)
        free = OccupancyEvidence(0, "a", 0, False, .95)
        occupied = replace(free, source_id="b", occupied=True)
        self.assertEqual(evaluate((free,), 0, config).state, CellState.FREE)
        self.assertAlmostEqual(evaluate((free, occupied), 0, config).probability, .5)
        many = tuple(replace(occupied, source_id=str(i)) for i in range(100))
        self.assertEqual(evaluate(many, 0, config).log_odds, 4)
        for bad in (.5, 1., float("nan")):
            with self.assertRaises(ValueError):
                replace(free, reliability=bad)

    def test_union_relay_loop_and_order(self):
        a, b, c = [ProbabilisticKnowledgeMap(7, 7) for _ in range(3)]
        p = Position(2, 2)
        a.observe({p: CellState.FREE}, step=0, source_id="a")
        b.observe({p: CellState.OCCUPIED}, step=0, source_id="b")
        c.apply(a.records)
        c.apply(b.records)
        a.apply(c.records)
        b.apply(a.records)
        c.apply(b.records)
        self.assertEqual(a.records, b.records)
        self.assertEqual(b.records, c.records)
        self.assertEqual(c.probability_at(p).evidence_count, 2)
        self.assertEqual(c.probability_at(p).probability, .5)
        self.assertEqual(merge_knowledge_maps([a, b]).records, c.records)

    def test_decay_delayed_receive_does_not_rejuvenate(self):
        a, b = [ProbabilisticKnowledgeMap(7, 7) for _ in range(2)]
        p = Position(1, 1)
        a.observe({p: CellState.OCCUPIED}, step=0, source_id="a")
        packet = a.records
        a.advance(80)
        b.advance(80)
        b.apply(packet)
        self.assertEqual(a.probability_at(p), b.probability_at(p))
        before = b.probability_at(p)
        b.apply(packet)
        self.assertEqual(before, b.probability_at(p))
        b.advance(640)
        b.apply(packet)
        self.assertEqual(b.records, ())
        self.assertEqual(b.probability_at(p).probability, .5)

    def test_dynamic_reopening_uses_only_observations(self):
        m = ProbabilisticKnowledgeMap(7, 7)
        p = Position(3, 3)
        m.observe({p: CellState.OCCUPIED}, step=0, source_id="a")
        for step in range(1, 4):
            m.observe({p: CellState.FREE}, step=step, source_id="a")
        self.assertTrue(m.is_known_free(p))

    def test_floor_identity(self):
        maps = {f: ProbabilisticKnowledgeMap(7, 7, floor=f) for f in (0, 1)}
        floors = MultiFloorKnowledge({0: 7, 1: 7}, {0: 7, 1: 7}, probabilistic=maps)
        maps[0].observe({Position(1, 1): CellState.FREE}, step=0, source_id="a")
        maps[1].apply(maps[0].records)
        self.assertTrue(floors.is_known_free(GridPosition(0, 1, 1)))
        self.assertEqual(floors.cell_at(GridPosition(1, 1, 1)), CellState.UNKNOWN)

    def test_sensor_reproducible_independent_of_call_order(self):
        cells = {Position(x, 1): CellState.FREE for x in range(7)}
        kwargs = dict(seed=3, agent_id="a", step=2, profile="high_noise")
        first = uncertain_observations(cells, **kwargs)
        self.assertEqual(first, uncertain_observations(dict(reversed(list(cells.items()))), **kwargs))

    def test_survivor_no_duplicate_and_contradiction(self):
        tracker = SurvivorHypothesisTracker(minimum_positive_observations=2,
            confirmation_threshold=.65, rejection_threshold=.12, negative_evidence_weight=.55)
        p = Position(2, 2)
        kwargs = dict(confidence=.8, channel="visual", agent_id="a", step=0)
        tracker.positive(p, **kwargs)
        tracker.positive(p, **kwargs)
        self.assertEqual(tracker.hypotheses[p].positive_observations, 1)
        tracker.negative(p, **dict(kwargs, step=1))
        self.assertLess(tracker.hypotheses[p].accumulated_evidence, .4)
        tracker.positive(p, **dict(kwargs, step=2))
        tracker.positive(p, **dict(kwargs, step=3))
        self.assertIn(p, tracker.confirmed_locations)
        tracker.negative(p, **dict(kwargs, step=4))
        self.assertIn(p, tracker.confirmed_locations)

    def test_scoring_without_world(self):
        m = ProbabilisticKnowledgeMap(7, 7, planning_variant="uncertainty-aware")
        p = Position(3, 3)
        self.assertEqual(m.information_bonus(p), 4)
        m.observe({n: CellState.FREE for n in p.neighbors()}, step=0, source_id="a")
        self.assertLess(m.information_bonus(p), 4)
        m.planning_variant = "naive"
        self.assertEqual(m.information_bonus(p), 0)


class ProbabilisticIntegrationTests(unittest.TestCase):
    def test_delayed_relay_transport_and_ttl(self):
        from test_network_transport import transport, snapshot
        from echorescue.network_transport import MessageType
        a, b = [ProbabilisticKnowledgeMap(7,7) for _ in range(2)]
        p = Position(2,2)
        a.observe({p:CellState.FREE},step=0,source_id="drone-1")
        net = transport()
        radio = snapshot(("drone-1","drone-2"),("drone-2","base"))
        net.enqueue(sender="drone-1",recipient="base",route=("drone-1","drone-2","base"),
            message_type=MessageType.MAP_UPDATE,payload=a.records,created_step=0,ttl=10)
        deliveries = []
        for step in range(5):
            b.advance(step)
            for delivery in net.advance(step=step,snapshot=radio):
                deliveries.append(delivery)
                b.apply(delivery.payload)
                b.apply(delivery.payload)
        self.assertTrue(deliveries)
        self.assertTrue(deliveries[0].relayed)
        self.assertEqual(b.probability_at(p).evidence_count,1)
        self.assertEqual(b.probability_at(p).observed_step,0)
        expired = transport(latency_steps=4)
        expired.enqueue(sender="drone-1",recipient="base",route=("drone-1","drone-2","base"),
            message_type=MessageType.MAP_UPDATE,payload=a.records,created_step=0,ttl=1)
        self.assertFalse([d for tick in range(8) for d in expired.advance(step=tick,snapshot=radio)])

    def test_full_mission_replay_reproducibility(self):
        from echorescue.config import SimulationConfig
        from echorescue.replay import generate_replay, replay_json_bytes
        config = SimulationConfig(seed=1,width=9,height=7,drone_count=2,
            uncertainty_profile="medium_noise",planning_variant="uncertainty-aware",max_steps=40)
        first, second = generate_replay(config), generate_replay(config)
        self.assertEqual(replay_json_bytes(first),replay_json_bytes(second))
        self.assertIn("probabilistic_maps",first["frames"][0])
        self.assertEqual(first["frames"][0]["planning_variant"],"uncertainty-aware")

    def test_local_confidence_is_not_shared_before_messages(self):
        from echorescue.config import SimulationConfig
        from echorescue.multi_simulation import MultiDroneSimulation
        sim = MultiDroneSimulation(SimulationConfig(width=9,height=7,drone_count=2,
            uncertainty_profile="clean",knowledge_mode="local",network_profile="constrained"))
        self.assertIsNot(sim.local_hypothesis_trackers["drone-1"], sim.local_hypothesis_trackers["drone-2"])
        self.assertFalse(sim.hypothesis_tracker.hypotheses)

    def test_floor_probabilistic_mission(self):
        from echorescue.multi_floor import MultiFloorSimulation, MultiFloorConfig
        sim = MultiFloorSimulation(MultiFloorConfig(seed=0,width=9,height=7,floor_count=2,
            uncertainty_profile="clean",max_steps=80))
        sim.run()
        self.assertIn("probabilistic_maps",sim.frames[-1])
        self.assertEqual(set(sim.frames[-1]["probabilistic_maps"]),{"0","1"})
