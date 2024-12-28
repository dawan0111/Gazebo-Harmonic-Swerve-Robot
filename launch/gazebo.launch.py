from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
# from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the package directory and robot description file
    share_dir = get_package_share_directory('amr_description')
    # controller_share_dir = get_package_share_directory('amr_control')  # Adjust if needed

    # Process the XACRO file to get the URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'amr.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    config_file = os.path.join(
        get_package_share_directory('amr_description'),
        'config',
        'ignition.yaml'
    )
    # Path to controller config file
    # controller_config_file = os.path.join(share_dir, 'config', 'controller.yaml')
    # world_file_arg = DeclareLaunchArgument(
    #     "world_file",
    #     default_value="worlds/empty.sdf",
    #     description="Path to the Gazebo world file."
    # )

    # Paths to robot description and world files
    # robot_description_file = PathJoinSubstitution([
    #     FindPackageShare("amr_des"),
    #     LaunchConfiguration("robot_description_file")
    # ])

    # world_file = PathJoinSubstitution([
    #     FindPackageShare("your_package_name"),
    #     LaunchConfiguration("world_file")
    # ])

    # Include gz_sim launch file
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "ign_args":"-r"
        }.items(),
    )

    # Spawn robot node
    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-file", xacro_file,
            "-x", "0",
            "-y", "0",
            "-z", "0"
        ],
        output="screen"
    )
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[f'--config {config_file}'],
        output='screen',
    )
    # gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzserver.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'pause': 'true'
    #     }.items()
    # )

    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzclient.launch.py'
    #         ])
    #     ])
    # )

    # urdf_spawn_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'amr',
    #         '-topic', 'robot_description'
    #     ],
    #     output='screen'
    # )
    config = PathJoinSubstitution(
        [
            FindPackageShare("amr_description"),
            "config",
            "controller.yaml",
        ]
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            # {'robot_description': robot_urdf},
            config
              # Load the controller configuration file
        ],
        output='screen'
    )

    

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_velocity_controller"],
    )

    steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_steering_controller"],
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    
    # # Load controllers using the controller_manager spawner
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_steering_controllers = [
    #     ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
    #         output='screen'
    #     )
    #     for controller in [
    #         'front_left_steer_controller',
    #         'front_right_steer_controller',
    #         'rear_left_steer_controller',
    #         'rear_right_steer_controller'
    #     ]
    # ]

    # load_drive_controllers = [
    #     ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
    #         output='screen'
    #     )
    #     for controller in [
    #         'front_left_drive_controller',
    #         'front_right_drive_controller',
    #         'rear_left_drive_controller',
    #         'rear_right_drive_controller'
    #     ]
    # ]

    # Return the complete launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        # gazebo_server,
        # gazebo_client,
        # urdf_spawn_node,
        controller_manager_node,
        velocity_controller,
        steering_controller,
        joint_broad_spawner,
        ignition_launch,
        spawn_robot,
        bridge_node,
        # joint_broad_spawner  # Add the controller manager
        # load_joint_state_broadcaster,
        # *load_steering_controllers,
        # *load_drive_controllers
    ])
