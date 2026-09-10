import unittest

from echorescue.flight_mission import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    CommandKind,
    FlightMissionController,
    MissionConfig,
    MissionPhase,
    validate_simulation_endpoint,
)
from echorescue.mavlink_telemetry import TelemetryHealth


SECOND = 1_000_000_000


class FlightMissionControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MissionConfig(
            target_altitude_enu_m=2.0,
            altitude_tolerance_m=0.25,
            hover_duration_s=2.0,
            preflight_hold_s=1e-9,
            ready_timeout_s=5.0,
            transition_timeout_s=3.0,
            takeoff_timeout_s=8.0,
            landing_timeout_s=8.0,
            cleanup_timeout_s=5.0,
        )
        self.controller = FlightMissionController(self.config, 0)

    def fresh(self, now_ns: int, *, session: str = "session-1") -> None:
        self.controller.observe_status(
            session_id=session,
            health=TelemetryHealth.CONNECTED,
            telemetry_age_s=0.1,
            now_ns=now_ns,
        )

    def ready(self, now_ns: int = SECOND) -> None:
        self.fresh(now_ns)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="STABILIZE", now_ns=now_ns
        )
        self.controller.observe_position(
            session_id="session-1", altitude_enu_m=0.2, now_ns=now_ns
        )
        self.controller.observe_landed(session_id="session-1", landed=True, now_ns=now_ns)
        self.assertIsNone(self.controller.tick(now_ns))
        self.assertIsNone(self.controller.tick(now_ns + 1))
        stream = self.issue_and_accept(CommandKind.EXTENDED_STATE_STREAM, now_ns + 2)
        self.assertEqual(stream.command_id, MAV_CMD_SET_MESSAGE_INTERVAL)
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=now_ns + 4
        )
        self.controller.tick(now_ns + 4)

    def issue_and_accept(self, expected: CommandKind, now_ns: int):
        request = self.controller.tick(now_ns)
        self.assertIsNotNone(request)
        self.assertEqual(request.kind, expected)
        self.assertTrue(
            self.controller.acknowledge(
                session_id="session-1",
                command_id=request.command_id,
                result=0,
                now_ns=now_ns + 1,
            )
        )
        return request

    def reach_hover(self) -> int:
        self.ready()
        guided_time = SECOND + 10
        guided = self.issue_and_accept(CommandKind.GUIDED, guided_time)
        self.assertEqual(guided.command_id, MAV_CMD_DO_SET_MODE)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=guided_time + 2
        )
        self.controller.tick(guided_time + 2)

        arm_time = guided_time + 3
        arm = self.issue_and_accept(CommandKind.ARM, arm_time)
        self.assertEqual(arm.command_id, MAV_CMD_COMPONENT_ARM_DISARM)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=True, flight_mode="GUIDED", now_ns=arm_time + 2
        )
        self.controller.tick(arm_time + 2)

        takeoff_time = arm_time + 3
        takeoff = self.issue_and_accept(CommandKind.TAKEOFF, takeoff_time)
        self.assertEqual(takeoff.command_id, MAV_CMD_NAV_TAKEOFF)
        self.assertEqual(takeoff.target_altitude_enu_m, 2.0)
        self.controller.observe_position(
            session_id="session-1", altitude_enu_m=1.75, now_ns=takeoff_time + 2
        )
        self.fresh(takeoff_time + 2)
        self.controller.tick(takeoff_time + 2)
        self.assertEqual(self.controller.phase, MissionPhase.HOVER)
        return takeoff_time + 2

    def test_complete_mission_requires_each_ack_and_later_telemetry_transition(self) -> None:
        hover_time = self.reach_hover()
        self.controller.tick(hover_time + 2 * SECOND)
        land_time = hover_time + 2 * SECOND + 1
        land = self.issue_and_accept(CommandKind.LAND, land_time)
        self.assertEqual(land.command_id, MAV_CMD_NAV_LAND)

        # An accepted LAND is not completion evidence.
        self.controller.tick(land_time + 2)
        self.assertEqual(self.controller.phase, MissionPhase.WAIT_LANDED_DISARMED)
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=land_time + 3
        )
        self.controller.tick(land_time + 3)
        self.assertEqual(self.controller.phase, MissionPhase.WAIT_LANDED_DISARMED)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="LAND", now_ns=land_time + 4
        )
        self.controller.tick(land_time + 4)

        self.assertTrue(self.controller.succeeded)
        report = self.controller.report()
        self.assertEqual([item["kind"] for item in report["commands"]], [
            "extended_state_stream", "guided", "arm", "takeoff", "land"
        ])
        self.assertTrue(all(item["ack_result"] == 0 for item in report["commands"]))
        self.assertTrue(report["final_landed"])
        self.assertFalse(report["final_armed"])

    def test_ack_alone_cannot_advance_guided_arm_or_takeoff(self) -> None:
        self.ready()
        self.issue_and_accept(CommandKind.GUIDED, SECOND + 10)
        self.controller.tick(SECOND + 11)
        self.assertEqual(self.controller.phase, MissionPhase.WAIT_GUIDED)

    def test_command_rejection_before_arm_fails_without_cleanup_command(self) -> None:
        self.ready()
        request = self.controller.tick(SECOND + 10)
        self.controller.acknowledge(
            session_id="session-1",
            command_id=request.command_id,
            result=4,
            now_ns=SECOND + 11,
        )
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("rejected", self.controller.report()["failure_reason"])

    def test_takeoff_timeout_while_armed_uses_acked_recovery_land(self) -> None:
        self.ready()
        guided_time = SECOND + 10
        self.issue_and_accept(CommandKind.GUIDED, guided_time)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=guided_time + 2
        )
        self.controller.tick(guided_time + 2)
        arm_time = guided_time + 3
        self.issue_and_accept(CommandKind.ARM, arm_time)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=True, flight_mode="GUIDED", now_ns=arm_time + 2
        )
        self.controller.tick(arm_time + 2)
        takeoff_time = arm_time + 3
        self.issue_and_accept(CommandKind.TAKEOFF, takeoff_time)

        timeout_time = takeoff_time + 9 * SECOND
        self.controller.tick(timeout_time)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_SEND_LAND)
        recovery = self.controller.tick(timeout_time + 1)
        self.assertTrue(recovery.recovery)
        self.assertEqual(recovery.kind, CommandKind.LAND)
        self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=timeout_time + 2,
        )
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=timeout_time + 3
        )
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="LAND", now_ns=timeout_time + 4
        )
        self.controller.tick(timeout_time + 4)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("recovery landing verified", self.controller.report()["failure_reason"])

    def test_stale_telemetry_while_armed_waits_for_freshness_before_cleanup(self) -> None:
        hover_time = self.reach_hover()
        self.controller.observe_status(
            session_id="session-1",
            health=TelemetryHealth.STALE,
            telemetry_age_s=1.1,
            now_ns=hover_time + 1,
        )
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_TELEMETRY)
        self.assertIsNone(self.controller.tick(hover_time + 2))
        self.fresh(hover_time + 3)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=True, flight_mode="GUIDED", now_ns=hover_time + 4
        )
        self.controller.tick(hover_time + 4)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_SEND_LAND)
        recovery_land = self.controller.tick(hover_time + 5)
        self.assertEqual(recovery_land.kind, CommandKind.LAND)
        self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=hover_time + 6,
        )
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=hover_time + 7
        )
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="LAND", now_ns=hover_time + 8
        )
        self.controller.tick(hover_time + 8)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("recovery landing verified", self.controller.report()["failure_reason"])

    def test_disconnect_after_arm_send_treats_arm_state_as_uncertain(self) -> None:
        self.ready()
        guided_time = SECOND + 10
        self.issue_and_accept(CommandKind.GUIDED, guided_time)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=guided_time + 2
        )
        self.controller.tick(guided_time + 2)
        self.controller.tick(guided_time + 3)
        self.controller.observe_status(
            session_id="session-1",
            health=TelemetryHealth.DISCONNECTED,
            telemetry_age_s=3.1,
            now_ns=guided_time + 4,
        )
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_TELEMETRY)

        self.fresh(guided_time + 5, session="session-2")
        self.controller.observe_heartbeat(
            session_id="session-2", armed=True, flight_mode="GUIDED", now_ns=guided_time + 6
        )
        self.controller.tick(guided_time + 6)
        recovery_land = self.controller.tick(guided_time + 7)
        self.assertEqual(recovery_land.kind, CommandKind.LAND)
        command_kinds = [item["kind"] for item in self.controller.report()["commands"]]
        self.assertEqual(command_kinds.count("arm"), 1)

    def test_ack_from_old_session_cannot_satisfy_pending_command(self) -> None:
        self.ready()
        request = self.controller.tick(SECOND + 10)
        self.assertFalse(self.controller.acknowledge(
            session_id="session-old",
            command_id=request.command_id,
            result=0,
            now_ns=SECOND + 11,
        ))
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=SECOND + 12
        )
        self.controller.tick(SECOND + 12)
        self.assertEqual(self.controller.phase, MissionPhase.WAIT_GUIDED)
        self.assertTrue(self.controller.acknowledge(
            session_id="session-1",
            command_id=request.command_id,
            result=0,
            now_ns=SECOND + 13,
        ))
        self.controller.tick(SECOND + 13)
        self.assertEqual(self.controller.phase, MissionPhase.WAIT_GUIDED)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="GUIDED", now_ns=SECOND + 14
        )
        self.controller.tick(SECOND + 14)
        self.assertEqual(self.controller.phase, MissionPhase.SEND_ARM)

    def test_timed_out_land_is_not_reissued_and_still_requires_both_states(self) -> None:
        hover_time = self.reach_hover()
        self.controller.tick(hover_time + 2 * SECOND)
        land_time = hover_time + 2 * SECOND + 1
        land = self.controller.tick(land_time)
        self.assertEqual(land.kind, CommandKind.LAND)
        timeout_time = land_time + 9 * SECOND
        self.controller.tick(timeout_time)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_LANDED_DISARMED)
        self.assertIsNone(self.controller.tick(timeout_time + 1))
        self.assertEqual(
            [item["kind"] for item in self.controller.report()["commands"]].count("land"),
            1,
        )
        self.controller.acknowledge(
            session_id="session-1", command_id=MAV_CMD_NAV_LAND,
            result=0, now_ns=timeout_time + 2,
        )
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="LAND", now_ns=timeout_time + 3
        )
        self.controller.tick(timeout_time + 3)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_LANDED_DISARMED)
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=timeout_time + 4
        )
        self.controller.tick(timeout_time + 4)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("recovery landing verified", self.controller.report()["failure_reason"])

    def test_stale_telemetry_does_not_reissue_land_in_the_same_session(self) -> None:
        hover_time = self.reach_hover()
        self.controller.tick(hover_time + 2 * SECOND)
        land_time = hover_time + 2 * SECOND + 1
        land = self.controller.tick(land_time)
        self.assertEqual(land.kind, CommandKind.LAND)
        self.controller.observe_status(
            session_id="session-1",
            health=TelemetryHealth.STALE,
            telemetry_age_s=1.1,
            now_ns=land_time + 1,
        )
        self.assertEqual(
            self.controller.phase,
            MissionPhase.RECOVERY_WAIT_LANDED_DISARMED,
        )
        self.fresh(land_time + 2)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=True, flight_mode="LAND", now_ns=land_time + 3
        )
        self.assertIsNone(self.controller.tick(land_time + 3))
        self.assertEqual(
            [item["kind"] for item in self.controller.report()["commands"]].count("land"),
            1,
        )
        self.assertTrue(self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=land_time + 4,
        ))
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=land_time + 5
        )
        self.controller.observe_heartbeat(
            session_id="session-1", armed=False, flight_mode="LAND", now_ns=land_time + 6
        )
        self.controller.tick(land_time + 6)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("recovery landing verified", self.controller.report()["failure_reason"])

    def test_reconnect_during_armed_landing_recovery_restarts_per_session_land(self) -> None:
        hover_time = self.reach_hover()
        self.controller.observe_status(
            session_id="session-1",
            health=TelemetryHealth.STALE,
            telemetry_age_s=1.1,
            now_ns=hover_time + 1,
        )
        self.fresh(hover_time + 2)
        self.controller.observe_heartbeat(
            session_id="session-1", armed=True, flight_mode="GUIDED", now_ns=hover_time + 3
        )
        self.controller.tick(hover_time + 3)
        first_land = self.controller.tick(hover_time + 4)
        self.assertEqual(first_land.kind, CommandKind.LAND)
        self.assertTrue(self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=hover_time + 5,
        ))
        self.controller.observe_landed(
            session_id="session-1", landed=True, now_ns=hover_time + 6
        )

        reconnect_time = hover_time + 7
        self.controller.observe_status(
            session_id="session-2",
            health=TelemetryHealth.CONNECTED,
            telemetry_age_s=0.1,
            now_ns=reconnect_time,
        )
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_TELEMETRY)
        self.assertEqual(self.controller.phase_started_ns, reconnect_time)
        self.assertIsNone(self.controller.armed)
        self.assertIsNone(self.controller.landed)
        self.assertFalse(self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=hover_time + 8,
        ))

        self.controller.observe_heartbeat(
            session_id="session-2", armed=True, flight_mode="LAND", now_ns=hover_time + 9
        )
        self.controller.tick(hover_time + 9)
        second_land = self.controller.tick(hover_time + 10)
        self.assertEqual(second_land.kind, CommandKind.LAND)
        self.assertTrue(second_land.recovery)
        self.assertFalse(self.controller.acknowledge(
            session_id="session-1",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=hover_time + 11,
        ))

        # Even new-session state received before its ACK cannot complete LAND.
        self.controller.observe_landed(
            session_id="session-2", landed=True, now_ns=hover_time + 12
        )
        self.controller.observe_heartbeat(
            session_id="session-2", armed=False, flight_mode="LAND", now_ns=hover_time + 13
        )
        self.assertTrue(self.controller.acknowledge(
            session_id="session-2",
            command_id=MAV_CMD_NAV_LAND,
            result=0,
            now_ns=hover_time + 14,
        ))
        self.controller.tick(hover_time + 14)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_LANDED_DISARMED)

        self.controller.observe_landed(
            session_id="session-2", landed=True, now_ns=hover_time + 15
        )
        self.controller.tick(hover_time + 15)
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_LANDED_DISARMED)
        self.controller.observe_heartbeat(
            session_id="session-2", armed=False, flight_mode="LAND", now_ns=hover_time + 16
        )
        self.controller.tick(hover_time + 16)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        self.assertIn("recovery landing verified", self.controller.report()["failure_reason"])

        land_sessions = [
            item["session_id"]
            for item in self.controller.report()["commands"]
            if item["kind"] == "land"
        ]
        self.assertEqual(land_sessions, ["session-1", "session-2"])

    def test_reconnect_aborts_and_does_not_reuse_pre_reconnect_observations(self) -> None:
        hover_time = self.reach_hover()
        self.controller.observe_status(
            session_id="session-2",
            health=TelemetryHealth.DEGRADED,
            telemetry_age_s=-1.0,
            now_ns=hover_time + 1,
        )
        self.assertEqual(self.controller.phase, MissionPhase.RECOVERY_WAIT_TELEMETRY)
        self.assertIn("reconnect changed session", self.controller.report()["failure_reason"])

    def test_ready_timeout_and_invalid_configuration(self) -> None:
        self.controller.tick(6 * SECOND)
        self.assertEqual(self.controller.phase, MissionPhase.FAILED)
        with self.assertRaisesRegex(ValueError, "below target"):
            MissionConfig(target_altitude_enu_m=1.0, altitude_tolerance_m=1.0)

    def test_simulation_endpoint_accepts_port_boundaries(self) -> None:
        for endpoint in (
            "tcp:127.0.0.1:1",
            "tcp:127.0.0.1:5760",
            "tcp:127.0.0.1:65535",
        ):
            self.assertEqual(validate_simulation_endpoint(endpoint), endpoint)

    def test_simulation_endpoint_rejects_invalid_ports_and_transports(self) -> None:
        for endpoint in (
            "tcp:127.0.0.1:0",
            "tcp:127.0.0.1:65536",
            "tcp:127.0.0.1:99999",
            "tcp:127.0.0.1:",
            "tcp:127.0.0.1:-1",
            "tcp:127.0.0.1:1.5",
            "tcp:127.0.0.1:١",
            "udp:127.0.0.1:14550",
            "tcp:192.0.2.1:5760",
            "/dev/ttyUSB0",
        ):
            with self.assertRaisesRegex(ValueError, "loopback TCP SITL"):
                validate_simulation_endpoint(endpoint)

    def test_unexpected_and_predating_ack_are_ignored(self) -> None:
        self.ready()
        request = self.controller.tick(SECOND + 10)
        self.assertFalse(
            self.controller.acknowledge(
                session_id="session-1",
                command_id=MAV_CMD_NAV_LAND,
                result=0,
                now_ns=SECOND + 11,
            )
        )
        self.assertFalse(
            self.controller.acknowledge(
                session_id="session-1",
                command_id=request.command_id,
                result=0,
                now_ns=SECOND + 9,
            )
        )


if __name__ == "__main__":
    unittest.main()
