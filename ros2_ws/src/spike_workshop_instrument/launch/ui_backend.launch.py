from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Compatibility shim: actual UI backend launch moved to spike_workshop_ui_backend.
    include_new_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("spike_workshop_ui_backend"),
                    "launch",
                    "ui_backend.launch.py",
                ]
            )
        )
    )
    return LaunchDescription([include_new_launch])
