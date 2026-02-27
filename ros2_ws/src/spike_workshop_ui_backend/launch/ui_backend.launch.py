from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    launch_args = [
        DeclareLaunchArgument("enable_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
    ]

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

    edu_command_runner_node = Node(
        package="spike_workshop_instrument",
        executable="edu_command_runner_node",
        name="edu_command_runner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rosbridge")),
    )

    return LaunchDescription(
        launch_args + [rosbridge_node, rosapi_node, edu_command_runner_node]
    )
