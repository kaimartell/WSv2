from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    launch_args = [
        DeclareLaunchArgument("host_agent_url", default_value="http://host.docker.internal:8000"),
        DeclareLaunchArgument("poll_hz", default_value="2.0"),
        DeclareLaunchArgument("http_timeout_sec", default_value="1.5"),
        DeclareLaunchArgument("action_http_timeout_sec", default_value="8.0"),
        DeclareLaunchArgument("unreachable_log_throttle_sec", default_value="10.0"),
        DeclareLaunchArgument("consecutive_failures_to_mark_down", default_value="2"),
        DeclareLaunchArgument("consecutive_successes_to_mark_up", default_value="1"),
        DeclareLaunchArgument("queue_policy", default_value="latest"),
        DeclareLaunchArgument("execution_mode", default_value="host_score"),
        DeclareLaunchArgument("fifo_queue_size", default_value="32"),
        DeclareLaunchArgument("action_queue_size", default_value="64"),
        DeclareLaunchArgument("enable_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("participant_id", default_value="1"),
        DeclareLaunchArgument("name", default_value="instrument"),
        DeclareLaunchArgument("mode", default_value="pulse"),
        DeclareLaunchArgument("speed", default_value="0.6"),
        DeclareLaunchArgument("duration", default_value="1.0"),
        DeclareLaunchArgument("repeats", default_value="4"),
        DeclareLaunchArgument("bpm", default_value="120.0"),
        DeclareLaunchArgument("amplitude", default_value="0.6"),
        DeclareLaunchArgument("sweep_steps", default_value="8"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("sequence_file", default_value=""),
        DeclareLaunchArgument("pattern_file", default_value=""),
        DeclareLaunchArgument("allow_override", default_value="false"),
        DeclareLaunchArgument("cmd_topic", default_value="/spike/cmd"),
        DeclareLaunchArgument("action_topic", default_value="/spike/action"),
        DeclareLaunchArgument("done_topic", default_value="/done"),
        DeclareLaunchArgument("status_topic", default_value="/status"),
    ]

    hw_node = Node(
        package="spike_workshop_hw",
        executable="spike_hw_client_node",
        name="spike_hw_client_node",
        output="screen",
        parameters=[
            {
                "host_agent_url": LaunchConfiguration("host_agent_url"),
                "poll_hz": LaunchConfiguration("poll_hz"),
                "http_timeout_sec": LaunchConfiguration("http_timeout_sec"),
                "action_http_timeout_sec": LaunchConfiguration("action_http_timeout_sec"),
                "unreachable_log_throttle_sec": LaunchConfiguration("unreachable_log_throttle_sec"),
                "consecutive_failures_to_mark_down": LaunchConfiguration(
                    "consecutive_failures_to_mark_down"
                ),
                "consecutive_successes_to_mark_up": LaunchConfiguration(
                    "consecutive_successes_to_mark_up"
                ),
                "queue_policy": LaunchConfiguration("queue_policy"),
                "execution_mode": LaunchConfiguration("execution_mode"),
                "fifo_queue_size": LaunchConfiguration("fifo_queue_size"),
                "action_topic": LaunchConfiguration("action_topic"),
                "action_queue_size": LaunchConfiguration("action_queue_size"),
            }
        ],
    )

    instrument_node = Node(
        package="spike_workshop_instrument",
        executable="instrument_node",
        name="instrument_node",
        output="screen",
        parameters=[
            {
                "participant_id": LaunchConfiguration("participant_id"),
                "name": LaunchConfiguration("name"),
                "mode": LaunchConfiguration("mode"),
                "speed": LaunchConfiguration("speed"),
                "duration": LaunchConfiguration("duration"),
                "repeats": LaunchConfiguration("repeats"),
                "bpm": LaunchConfiguration("bpm"),
                "amplitude": LaunchConfiguration("amplitude"),
                "sweep_steps": LaunchConfiguration("sweep_steps"),
                "seed": LaunchConfiguration("seed"),
                "sequence_file": LaunchConfiguration("sequence_file"),
                "pattern_file": LaunchConfiguration("pattern_file"),
                "execution_mode": LaunchConfiguration("execution_mode"),
                "allow_override": LaunchConfiguration("allow_override"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "action_topic": LaunchConfiguration("action_topic"),
                "done_topic": LaunchConfiguration("done_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
            }
        ],
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rosbridge")),
        parameters=[
            {
                "address": LaunchConfiguration("rosbridge_address"),
                "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
            }
        ],
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rosbridge")),
    )

    return LaunchDescription(launch_args + [hw_node, instrument_node, rosbridge_node, rosapi_node])
