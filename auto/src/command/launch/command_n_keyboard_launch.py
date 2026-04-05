from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    key_mappings = LaunchConfiguration("key_mappings")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="key_mappings",
                default_value="",
                description="Keyboard mapping for keyboardjoy node",
            ),
            Node(
                package="keyboard_joy",
                executable="keyboard_joy",
                name="keyboard_input",
                output="screen",
                parameters=[key_mappings],
            ),
            Node(
                package="command",
                executable="command",
                name="command",
                output="screen",
            ),
        ]
    )
