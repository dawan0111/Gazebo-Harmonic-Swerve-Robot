from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = PathJoinSubstitution(
        [
            FindPackageShare("amr_description"),
            "config",
            "joystick.yaml",
        ]
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[config],
         )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[
                {'enable_button': -1},
                {'require_enable_button': False},
                {'axis_linear.x': 1},
                {'axis_linear.y': 0},
                {'axis_linear.z': 7},
                {'axis_angular.pitch': 7},
                {'axis_angular.roll': 7},
                {'axis_angular.yaw': 3},
                {'scale_linear.x': 4.0},
                {'scale_linear.y': -4.0},
                {'scale_linear.z': 1.0},
                {'scale_angular.pitch': 1.0},
                {'scale_angular.roll': 1.0},
                {'scale_angular.yaw': 1.0}
            ],
            )
#...

# And add to launch description at the bottom

    return LaunchDescription([
        joy_node,
        teleop_node       
    ])
