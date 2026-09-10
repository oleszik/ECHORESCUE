import json
import math
import unittest

from echorescue.flight_mission import MAV_CMD_NAV_LAND, MissionPhase
from echorescue.mavlink_telemetry import TelemetryHealth
from echorescue.waypoint_mission import (
    EnuTarget,
    NavigationPhase,
    POSITION_AND_YAW_TYPE_MASK,
    POSITION_ONLY_TYPE_MASK,
    RelativeTarget,
    TargetRequest,
    WaypointMissionConfig,
    WaypointMissionController,
    enu_target_to_ned,
    serialize_waypoint_report,
    target_errors,
)


def config(**overrides: object) -> WaypointMissionConfig:
    values = {
        "targets": (
            RelativeTarget("a", 3.0, 0.0, 2.0),
            RelativeTarget("b", 3.0, 3.0, 2.5),
            RelativeTarget("return-launch", 0.0, 0.0, 2.0),
        ),
        "settling_time_s": 1.0,
        "progress_timeout_s": 2.0,
        "target_timeout_s": 10.0,
        "mission_timeout_s": 100.0,
        "preflight_hold_s": 1.0,
        "ready_timeout_s": 3.0,
    }
    values.update(overrides)
    return WaypointMissionConfig(**values)  # type: ignore[arg-type]


def at_navigation(**overrides: object) -> WaypointMissionController:
    controller = WaypointMissionController(config(**overrides), 0)
    controller.flight.session_id = "session-1"
    controller.flight.health = TelemetryHealth.CONNECTED
    controller.flight.telemetry_age_s = 0.1
    controller.flight.armed = True
    controller.flight.flight_mode = "GUIDED"
    controller.flight.landed = False
    controller.flight.phase = MissionPhase.NAVIGATION_HOLD
    controller.launch_enu = (10.0, -4.0, 0.1)
    controller._position = (10.0, -4.0, 2.1)
    controller._position_session = "session-1"
    controller._position_source_ms = 100
    controller._position_receipt_ns = 1_000
    controller._build_next_target(1_000)
    return controller


def transmit(controller: WaypointMissionController, now_ns: int = 2_000) -> TargetRequest:
    request = controller.tick(now_ns)
    assert isinstance(request, TargetRequest)
    assert controller.target_transmitted(
        session_id="session-1", success=True, now_ns=now_ns + 1,
    )
    return request


class ConversionAndGeometryTests(unittest.TestCase):
    def test_enu_target_converts_to_absolute_local_ned(self) -> None:
        converted = enu_target_to_ned(EnuTarget("a", 3.0, 4.0, 2.5))
        self.assertEqual((converted.north_m, converted.east_m, converted.down_m), (4.0, 3.0, -2.5))
        self.assertEqual(converted.type_mask, POSITION_ONLY_TYPE_MASK)

    def test_optional_enu_heading_converts_to_ned_yaw(self) -> None:
        converted = enu_target_to_ned(EnuTarget("a", 0.0, 0.0, 1.0, 0.0))
        self.assertAlmostEqual(converted.yaw_rad, math.pi / 2.0)
        self.assertEqual(converted.type_mask, POSITION_AND_YAW_TYPE_MASK)

    def test_horizontal_vertical_and_total_error(self) -> None:
        horizontal, vertical, total = target_errors(EnuTarget("a", 3.0, 4.0, 2.0), 0.0, 0.0, 0.0)
        self.assertEqual(horizontal, 5.0)
        self.assertEqual(vertical, 2.0)
        self.assertAlmostEqual(total, math.sqrt(29.0))


class TargetCorrelationTests(unittest.TestCase):
    def test_takeoff_settling_clock_requires_horizontal_launch_tolerance(self) -> None:
        controller = WaypointMissionController(config(), 0)
        controller.flight.session_id = "session-1"
        controller.flight.health = TelemetryHealth.CONNECTED
        controller.flight.telemetry_age_s = 0.1
        controller.flight.armed = True
        controller.flight.flight_mode = "GUIDED"
        controller.flight.landed = False
        controller.flight.phase = MissionPhase.HOVER
        controller.flight.altitude_enu_m = 2.0
        controller.flight.hover_started_ns = 0
        controller.launch_enu = (10.0, -4.0, 0.0)
        controller.observe_position(
            session_id="session-1", east_m=10.5, north_m=-4.0, up_m=2.0,
            source_time_boot_ms=100, now_ns=800_000_000,
        )
        controller.tick(1_100_000_000)
        self.assertEqual(controller.flight.phase, MissionPhase.HOVER)
        controller.observe_position(
            session_id="session-1", east_m=10.1, north_m=-4.0, up_m=2.0,
            source_time_boot_ms=101, now_ns=900_000_000,
        )
        controller.tick(1_900_000_001)
        self.assertEqual(controller.flight.phase, MissionPhase.NAVIGATION_HOLD)

    def test_target_requires_fresh_active_session_before_send(self) -> None:
        controller = at_navigation()
        controller.flight.health = TelemetryHealth.STALE
        self.assertIsNone(controller.tick(2_000))
        self.assertEqual(controller.phase, NavigationPhase.LANDING)
        self.assertIn("fresh active-session", controller.report()["failure_reason"])

    def test_old_session_transmission_cannot_start_tracking(self) -> None:
        controller = at_navigation()
        request = controller.tick(2_000)
        self.assertIsInstance(request, TargetRequest)
        self.assertFalse(controller.target_transmitted(session_id="old", success=True, now_ns=2_001))
        self.assertEqual(controller.phase, NavigationPhase.WAIT_TRANSMISSION)

    def test_pretransmission_and_nonadvancing_vehicle_time_cannot_arrive(self) -> None:
        controller = at_navigation()
        transmit(controller)
        target = controller.active_target
        assert target is not None
        controller.observe_position(
            session_id="session-1", east_m=target.east_m, north_m=target.north_m,
            up_m=target.up_m, source_time_boot_ms=100, now_ns=3_000,
        )
        self.assertIsNone(controller.targets[-1].get("arrival_monotonic_ns"))

    def test_old_session_position_cannot_complete_current_target(self) -> None:
        controller = at_navigation()
        transmit(controller)
        target = controller.active_target
        assert target is not None
        controller.observe_position(
            session_id="old-session", east_m=target.east_m, north_m=target.north_m,
            up_m=target.up_m, source_time_boot_ms=999, now_ns=3_000,
        )
        self.assertEqual(controller.session_id, "session-1")
        self.assertIsNone(controller.targets[-1].get("first_post_command_monotonic_ns"))

    def test_arrival_requires_both_axes_and_continuous_settling(self) -> None:
        controller = at_navigation()
        transmit(controller)
        target = controller.active_target
        assert target is not None
        controller.observe_position(session_id="session-1", east_m=target.east_m, north_m=target.north_m, up_m=target.up_m + 0.5, source_time_boot_ms=101, now_ns=3_000)
        self.assertIsNone(controller.targets[-1].get("arrival_monotonic_ns"))
        controller.observe_position(session_id="session-1", east_m=target.east_m, north_m=target.north_m, up_m=target.up_m, source_time_boot_ms=102, now_ns=4_000)
        controller.observe_position(session_id="session-1", east_m=target.east_m + 1.0, north_m=target.north_m, up_m=target.up_m, source_time_boot_ms=103, now_ns=500_000_000)
        self.assertIsNone(controller.targets[-1].get("arrival_monotonic_ns"))

    def test_stationary_but_already_arrived_settles_without_progress(self) -> None:
        controller = at_navigation(targets=(RelativeTarget("here", 0.0, 0.0, 2.0),))
        transmit(controller)
        controller.observe_position(session_id="session-1", east_m=10.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=101, now_ns=1_000_003_000)
        controller.observe_position(session_id="session-1", east_m=10.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=102, now_ns=2_000_004_000)
        self.assertEqual(controller.targets[0]["outcome"], "settled")
        self.assertNotIn("time_to_first_progress_s", controller.targets[0])

    def test_progress_and_successful_settling_are_reported(self) -> None:
        controller = at_navigation(targets=(RelativeTarget("a", 3.0, 0.0, 2.0),))
        transmit(controller)
        controller.observe_position(session_id="session-1", east_m=11.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=101, now_ns=100_000_000)
        controller.observe_position(session_id="session-1", east_m=13.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=102, now_ns=200_000_000)
        controller.observe_position(session_id="session-1", east_m=13.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=103, now_ns=1_300_000_000)
        record = controller.targets[0]
        self.assertEqual(record["outcome"], "settled")
        self.assertGreater(record["time_to_first_progress_s"], 0.0)
        self.assertLessEqual(record["horizontal_error_m"], controller.config.horizontal_tolerance_m)

    def test_progress_timeout_aborts_to_one_land_recovery(self) -> None:
        controller = at_navigation(progress_timeout_s=1.0)
        transmit(controller)
        request = controller.tick(1_100_002_001)
        self.assertIsNotNone(request)
        assert request is not None
        self.assertEqual(request.command_id, MAV_CMD_NAV_LAND)
        controller.tick(1_100_002_002)
        lands = [item for item in controller.report()["commands"] if item["command_id"] == MAV_CMD_NAV_LAND]
        self.assertEqual(len(lands), 1)

    def test_target_timeout_is_explicit_and_enters_recovery(self) -> None:
        controller = at_navigation(target_timeout_s=1.0, progress_timeout_s=5.0)
        transmit(controller)
        request = controller.tick(1_100_002_001)
        self.assertIsNotNone(request)
        self.assertIn("target a timeout", controller.report()["failure_reason"])

    def test_stale_telemetry_during_navigation_stops_plan(self) -> None:
        controller = at_navigation()
        transmit(controller)
        controller.observe_status(
            session_id="session-1", health=TelemetryHealth.STALE,
            telemetry_age_s=2.0, now_ns=3_000,
        )
        controller.tick(3_001)
        self.assertEqual(controller.phase, NavigationPhase.LANDING)
        self.assertEqual(controller.targets[-1]["outcome"], "failed")

    def test_out_of_order_vehicle_time_aborts(self) -> None:
        controller = at_navigation()
        transmit(controller)
        controller.observe_position(session_id="session-1", east_m=11.0, north_m=-4.0, up_m=2.1, source_time_boot_ms=99, now_ns=3_000)
        self.assertEqual(controller.phase, NavigationPhase.LANDING)
        self.assertIn("vehicle-time regression", controller.report()["failure_reason"])

    def test_transmission_failure_aborts(self) -> None:
        controller = at_navigation()
        controller.tick(2_000)
        self.assertTrue(controller.target_transmitted(session_id="session-1", success=False, now_ns=2_001, detail="socket closed"))
        self.assertEqual(controller.phase, NavigationPhase.LANDING)


class ReconnectAndSafetyTests(unittest.TestCase):
    def test_disarmed_reconnect_before_takeoff_fails_without_arm_repeat(self) -> None:
        controller = WaypointMissionController(config(), 0)
        controller.flight.session_id = "one"
        controller.flight.armed = False
        controller.observe_status(session_id="two", health=TelemetryHealth.CONNECTED, telemetry_age_s=0.0, now_ns=10)
        self.assertEqual(controller.phase, NavigationPhase.FAILED)
        self.assertFalse(any(item["kind"] == "arm" for item in controller.report()["commands"]))

    def test_airborne_reconnect_invalidates_target_and_never_resumes(self) -> None:
        controller = at_navigation()
        transmit(controller)
        controller.observe_status(session_id="session-2", health=TelemetryHealth.CONNECTED, telemetry_age_s=0.0, now_ns=3_000)
        self.assertEqual(controller.phase, NavigationPhase.LANDING)
        self.assertEqual(controller.targets[-1]["outcome"], "invalidated")
        self.assertIsNone(controller.active_target)

    def test_old_session_ack_rejected_after_airborne_reconnect_and_one_land_per_session(self) -> None:
        controller = at_navigation()
        transmit(controller)
        controller.observe_status(session_id="session-2", health=TelemetryHealth.CONNECTED, telemetry_age_s=0.0, now_ns=3_000)
        controller.observe_heartbeat(session_id="session-2", armed=True, flight_mode="GUIDED", now_ns=3_100)
        self.assertIsNone(controller.tick(3_200))
        request = controller.tick(3_201)
        self.assertIsNotNone(request)
        assert request is not None
        self.assertEqual(request.command_id, MAV_CMD_NAV_LAND)
        self.assertFalse(controller.acknowledge(session_id="session-1", command_id=MAV_CMD_NAV_LAND, result=0, now_ns=3_300))
        controller.tick(3_400)
        lands = [item for item in controller.report()["commands"] if item["command_id"] == MAV_CMD_NAV_LAND and item["session_id"] == "session-2"]
        self.assertEqual(len(lands), 1)

    def test_reconnect_recovery_completes_only_after_land_ack_on_ground_and_disarmed(self) -> None:
        controller = at_navigation()
        transmit(controller)
        controller.observe_status(session_id="session-2", health=TelemetryHealth.CONNECTED, telemetry_age_s=0.0, now_ns=3_000)
        controller.observe_heartbeat(session_id="session-2", armed=True, flight_mode="GUIDED", now_ns=3_100)
        controller.tick(3_200)
        request = controller.tick(3_201)
        assert request is not None
        controller.acknowledge(session_id="session-2", command_id=MAV_CMD_NAV_LAND, result=0, now_ns=3_300)
        controller.observe_landed(session_id="session-2", landed=True, now_ns=3_400)
        controller.tick(3_500)
        self.assertFalse(controller.terminal)
        controller.observe_heartbeat(session_id="session-2", armed=False, flight_mode="LAND", now_ns=3_600)
        controller.tick(3_700)
        self.assertEqual(controller.phase, NavigationPhase.FAILED)
        self.assertIn("recovery landing verified", controller.flight.report()["failure_reason"])

    def test_geofence_boundaries_and_rejection(self) -> None:
        boundary = at_navigation(
            targets=(RelativeTarget("edge", 8.0, 0.0, 4.0),),
            geofence_horizontal_radius_m=8.0,
            geofence_max_altitude_m=4.0,
        )
        self.assertIsInstance(boundary.tick(2_000), TargetRequest)
        outside = at_navigation(targets=(RelativeTarget("outside", 8.01, 0.0, 2.0),))
        recovery = outside.tick(2_000)
        self.assertIsNotNone(recovery)
        self.assertFalse(outside.geofence_checks[-1]["passed"])

    def test_unexpected_mode_disarm_and_landing_abort_navigation(self) -> None:
        cases = (
            lambda item: item.observe_heartbeat(session_id="session-1", armed=True, flight_mode="LOITER", now_ns=2_000),
            lambda item: item.observe_heartbeat(session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=2_000),
            lambda item: item.observe_landed(session_id="session-1", landed=True, now_ns=2_000),
        )
        for action in cases:
            with self.subTest(action=action):
                controller = at_navigation()
                action(controller)
                self.assertIn(controller.phase, (NavigationPhase.LANDING, NavigationPhase.FAILED))

    def test_serialization_is_deterministic(self) -> None:
        controller = at_navigation()
        first = serialize_waypoint_report(controller.report())
        second = serialize_waypoint_report(controller.report())
        self.assertEqual(first, second)
        self.assertEqual(json.loads(first)["milestone"], "v0.14.4")


if __name__ == "__main__":
    unittest.main()
