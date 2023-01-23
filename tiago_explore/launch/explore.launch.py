from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    explore_cmd = Node(package='tiago_explore',
                       executable='explore',
                       output='screen',
                       remappings=[
                        ('input_scan', '/scan_raw'), 
                        ('motion_command', '/nav_vel')
                      ])

    ld = LaunchDescription()
    ld.add_action(explore_cmd)

    return ld