from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("width", default_value="11"),
        DeclareLaunchArgument("height", default_value="9"),
        DeclareLaunchArgument("survivor_count", default_value="2"),
        DeclareLaunchArgument("battery_capacity", default_value="300.0"),
        DeclareLaunchArgument("report_out", default_value="replays/v0.13-ros2-report.json"),
        DeclareLaunchArgument("replay_out", default_value="replays/v0.13-ros2-replay.json"),
        DeclareLaunchArgument("command_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("data_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("drop_observations_after", default_value="-1"),
        DeclareLaunchArgument("drop_states_after", default_value="-1"),
        DeclareLaunchArgument("duplicate_messages", default_value="false"),
        DeclareLaunchArgument("publish_delayed_messages", default_value="false"),
    ]
    shared = {
        "width": LaunchConfiguration("width"),
        "height": LaunchConfiguration("height"),
        "survivor_count": LaunchConfiguration("survivor_count"),
        "battery_capacity": LaunchConfiguration("battery_capacity"),
    }
    simulator = Node(
        package="echorescue_ros",
        executable="simulator_node",
        name="echorescue_simulator",
        output="screen",
        parameters=[
            shared,
            {
                "seed": LaunchConfiguration("seed"),
                "drop_observations_after": LaunchConfiguration("drop_observations_after"),
                "drop_states_after": LaunchConfiguration("drop_states_after"),
                "duplicate_messages": LaunchConfiguration("duplicate_messages"),
                "publish_delayed_messages": LaunchConfiguration("publish_delayed_messages"),
            },
        ],
    )
    mission = Node(
        package="echorescue_ros",
        executable="mission_node",
        name="echorescue_mission",
        output="screen",
        parameters=[
            shared,
            {
                "report_out": LaunchConfiguration("report_out"),
                "replay_out": LaunchConfiguration("replay_out"),
                "command_timeout_s": LaunchConfiguration("command_timeout_s"),
                "data_timeout_s": LaunchConfiguration("data_timeout_s"),
            },
        ],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason="mission process exited"))],
        )
    )
    return LaunchDescription([*arguments, simulator, mission, shutdown])
