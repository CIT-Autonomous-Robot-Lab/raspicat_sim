#!/usr/bin/env python3
"""
Raspicat with Livox MID-360 Gazebo Launch File

Launches Raspicat robot in Gazebo with:
- ISCAS Museum world environment
- Livox MID-360 LiDAR sensor
- Differential drive control
- Camera and IMU sensors

Robot control:
  - Subscribe: /cmd_vel (geometry_msgs/Twist)
  - Publish: /odom (nav_msgs/Odometry)
  - Publish: /livox/lidar (sensor_msgs/PointCloud2)

Usage:
  ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py [x_pose:=0.0] [y_pose:=0.0]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.events import matches_action, Shutdown
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Package directories
    pkg_raspicat_gazebo = get_package_share_directory('raspicat_gazebo')
    pkg_raspicat_description = get_package_share_directory('raspicat_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_livox_simulation = get_package_share_directory('ros2_livox_simulation')

    # Paths
    world_file = os.path.join(pkg_raspicat_gazebo, 'worlds', 'iscas_museum.world')
    urdf_file = os.path.join(pkg_raspicat_description, 'urdf', 'raspicat_with_livox.urdf.xacro')

    # Set GAZEBO_MODEL_PATH to include raspicat_description and ros2_livox_simulation packages
    gazebo_model_path = os.path.join(pkg_raspicat_description, '..') + ':' + os.path.join(pkg_ros2_livox_simulation, '..')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    verbose = LaunchConfiguration('verbose', default='false')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='false')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true for verbose Gazebo output'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of robot'
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': verbose
        }.items()
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'raspicat',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen'
    )

    # Raspicat simulation node (Lifecycle Node)
    try:
        raspicat_param_file = os.path.join(
            get_package_share_directory('raspicat'), 'config', 'raspicat.param.yaml'
        )
        if not os.path.exists(raspicat_param_file):
            print(f"Warning: Config file not found at {raspicat_param_file}, using default parameters")
            raspicat_params = [{'use_sim_time': use_sim_time}]
        else:
            raspicat_params = [raspicat_param_file, {'use_sim_time': use_sim_time}]
    except Exception as e:
        print(f"Warning: Could not load raspicat config: {e}, using default parameters")
        raspicat_params = [{'use_sim_time': use_sim_time}]

    raspicat_sim_node = LifecycleNode(
        namespace='',
        name='raspimouse',
        package='raspicat_gazebo',
        executable='raspicat_sim',
        output='screen',
        parameters=raspicat_params
    )

    # Emit events to configure and activate the lifecycle node
    emit_configuring_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(raspicat_sim_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_activating_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(raspicat_sim_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    emit_shutdown_event = EmitEvent(
        event=Shutdown()
    )

    register_activating_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=raspicat_sim_node,
            goal_state='inactive',
            entities=[
                emit_activating_event
            ],
        )
    )

    register_shutting_down_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=raspicat_sim_node,
            goal_state='finalized',
            entities=[
                emit_shutdown_event
            ],
        )
    )

    # RViz2 (optional)
    rviz_config_file = os.path.join(pkg_raspicat_gazebo, 'launch', 'config', 'gazebo.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        # Set DISPLAY for GUI
        SetEnvironmentVariable('DISPLAY', ':0'),
        # Disable Gazebo online model database to speed up startup
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # Set GAZEBO_MODEL_PATH to find raspicat_description meshes and livox meshes
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        # Declare arguments
        declare_use_sim_time,
        declare_verbose,
        declare_gui,
        declare_rviz,
        declare_x_pose,
        declare_y_pose,
        # Launch nodes
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        # Lifecycle node and its event handlers
        raspicat_sim_node,
        register_activating_transition,
        register_shutting_down_transition,
        emit_configuring_event,
        rviz2,
    ])
