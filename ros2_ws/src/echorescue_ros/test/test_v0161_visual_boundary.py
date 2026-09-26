from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ROS = ROOT / "ros2_ws/src/echorescue_ros/echorescue_ros"
INTERFACES = ROOT / "ros2_ws/src/echorescue_interfaces/msg"


def test_visual_interfaces_are_typed_and_versioned():
    names = (
        "VisualCandidateObservation.msg", "SurvivorEvidenceUpdate.msg",
        "ConfirmedSurvivor.msg", "PerceptionStatus.msg", "PerceptionMissionSummary.msg",
    )
    for name in names:
        text = (INTERFACES / name).read_text()
        assert "schema_version" in text
        assert "session_id" in text


def test_production_perception_has_no_ground_truth_or_gazebo_world_access():
    sources = "\n".join((ROS / name).read_text() for name in (
        "visual_survivor_perception.py", "mavlink_frontier_exploration.py",
    ))
    prohibited = ("ground_truth", "model/pose", "/world/", "gazebo_evaluation", "truth_id")
    assert not any(value in sources for value in prohibited)


def test_observer_remains_dependency_light():
    source = (ROS / "visual_search_observer.py").read_text()
    assert "pymavlink" not in source
    assert "cv2" not in source
    assert "gz." not in source
    assert "/world/" not in source


def test_camera_boundary_uses_sensor_msgs_and_bounded_qos():
    source = (ROS / "gazebo_camera_bridge.py").read_text()
    qos = (ROS / "qos.py").read_text()
    assert "sensor_msgs.msg import CameraInfo, Image" in source
    assert "IMAGE_QOS" in source
    assert "depth=2" in qos


def test_evaluator_truth_is_not_a_controller_configuration_dependency():
    config = (ROOT / "config/visual-survivor-v0.16.1.json").read_text()
    assert "truth-alpha" not in config
    assert "ground-truth" not in config
    assert '"survivor_count_available": false' in config
    assert '"survivor_positions_available": false' in config


def test_legacy_bridge_remains_receive_only():
    source = (ROS / "mavlink_telemetry_bridge.py").read_text().lower()
    assert "command_long_send" not in source
    assert "set_position_target" not in source
    assert "rc_channels_override" not in source
