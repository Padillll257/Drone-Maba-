import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    key_mappings_path = os.path.join(
        get_package_share_directory('curses_keyboard_joy'),
        'config',
        'key_mappings.yaml'
    )

    return LaunchDescription([
        Node(
            package='curses_keyboard_joy',
            executable='joy_node',
            name='curses_keyboard_joy',
            output='screen',
            # Launch the node with a terminal prefix because curses requires an actual terminal.
            prefix='xterm -e',
            parameters=[
                {'config': key_mappings_path}
            ]
        )
    ])
