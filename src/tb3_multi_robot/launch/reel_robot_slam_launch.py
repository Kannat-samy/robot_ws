#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    # =========================
    # Robots : robot1 = leader (Nav2), robot2 = follower (pas de Nav2)
    # =========================
    robots = [
        {'name': 'robot1', 'x_pose': '-2', 'y_pose': '-0.5', 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': '-2', 'y_pose': '0.5', 'z_pose': 0.01},
    ]

    TURTLEBOT3_MODEL = 'burger'

    # =========================
    # Launch arguments
    # =========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=use_sim_time,
        description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive',
        default_value=enable_drive,
        description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz',
        default_value=enable_rviz,
        description='Enable rviz launch'
    )

    # =========================
    # Packages / paths
    # =========================
    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params_slam.yaml'),
        description='Full path to the ROS2 parameters file'
    )

    # =========================
    # Map server UNIQUE (partagé entre les deux robots)
    # =========================
    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)

    # =========================
    # Spawn robots (séquentiel via RegisterEventHandler)
    # =========================
    last_action = None

    for robot in robots:
        namespace = ['/' + robot['name']]

        # RSP pour chaque robot
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 10.0
            }],
            remappings=remappings_tf,
            arguments=[urdf],
        )

        # Nav2 seulement pour robot1
        if robot['name'] == 'robot1':
            bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam': 'True',
                    'namespace': namespace,
                    'use_namespace': 'True',
                    'map': '',
                    'map_server': 'False',
                    'params_file': params_file,
                    'default_bt_xml_filename': os.path.join(
                        get_package_share_directory('nav2_bt_navigator'),
                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
                    ),
                    'autostart': 'true',
                    'use_sim_time': use_sim_time,
                    'log_level': 'warn'
                }.items()
            )
            actions = [turtlebot_state_publisher, bringup_cmd]
        else:
            # robot2 : juste spawn + RSP, pas de Nav2
            actions = [turtlebot_state_publisher]

        if last_action is None:
            for action in actions:
                ld.add_action(action)
        else:
            spawn_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=actions,
                )
            )
            ld.add_action(spawn_event)

        last_action = turtlebot_state_publisher

    # =========================
    # RViz pour robot1
    # =========================
    robot1 = robots[0]
    namespace_robot1 = ['/' + robot1['name']]

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace_robot1,
            'use_namespace': 'True',
            'rviz_config': rviz_config_file,
            'log_level': 'warn'
        }.items(),
        condition=IfCondition(enable_rviz)
    )

    ld.add_action(rviz_cmd)

    return ld