import unittest

from echorescue.environment import GridWorld
from echorescue.events import EventType, MissionEvent, MissionLog
from echorescue.models import Position
from echorescue.survivors import SurvivorSensor


class SurvivorSensorTests(unittest.TestCase):
    def test_range_and_wall_occlusion_are_enforced(self) -> None:
        visible_survivor = Position(2, 2)
        blocked_survivor = Position(4, 2)
        out_of_range_survivor = Position(1, 6)
        world = GridWorld(
            width=8,
            height=8,
            base=Position(1, 1),
            walls=frozenset({Position(3, 2)}),
            survivors=frozenset(
                {visible_survivor, blocked_survivor, out_of_range_survivor}
            ),
        )

        observed = SurvivorSensor(max_range=3).observe(world, Position(1, 2))

        self.assertEqual(observed, (visible_survivor,))


class MissionLogTests(unittest.TestCase):
    def test_duplicate_survivor_events_are_rejected(self) -> None:
        log = MissionLog()
        position = Position(4, 3)
        detected = MissionEvent(
            position, step=5, drone_id="drone-1", event_type=EventType.SURVIVOR_DETECTED
        )
        confirmed = MissionEvent(
            position,
            step=6,
            drone_id="drone-1",
            event_type=EventType.SURVIVOR_CONFIRMED,
        )

        self.assertTrue(log.record(detected))
        self.assertFalse(log.record(detected))
        self.assertTrue(log.record(confirmed))
        self.assertFalse(log.record(confirmed))
        self.assertEqual(log.events, (detected, confirmed))


if __name__ == "__main__":
    unittest.main()
