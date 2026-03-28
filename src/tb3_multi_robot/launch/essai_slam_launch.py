#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Adapté depuis Arshad Mehmood
# 2 robots : robot1 (Nav2 leader) + robot2 (follower, pas de Nav2)
# Un seul RViz pour robot1

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
        {'name': 'robot2', 'x_pose': '-2', 'y_pose': '0.5',  'z_pose': 0.01},
    ]

    TURTLEBOT3_MODEL = 'burger'

    # =========================
    # Launch arguments
    # =========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
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

    world = os.path.join(
        package_dir, 'worlds', 'multi_robot_world.world'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )

    # =========================
    # Gazebo
    # =========================
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    # =========================
    # Map server UNIQUE (partagé entre les deux robots)
    # =========================
    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(
                get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'
            ),
        }],
        remappings=remappings_tf
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    #ld.add_action(map_server)
    #ld.add_action(map_server_lifecycle)

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

        # Spawn Gazebo
        spawn_turtlebot3 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(package_dir, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', str(robot['z_pose']),
                '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        # Nav2 seulement pour robot1
        if robot['name'] == 'robot1':
            bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam': 'True', #False pour enlever slam
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
            actions = [spawn_turtlebot3, turtlebot_state_publisher, bringup_cmd]
        else:
            # robot2 : juste spawn + RSP, pas de Nav2
            actions = [spawn_turtlebot3, turtlebot_state_publisher]

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

        last_action = spawn_turtlebot3

    # =========================
    # Pose initiale + RViz (après le dernier spawn)
    # UN SEUL RViz pour robot1
    # =========================
    robot1 = robots[0]
    namespace_robot1 = ['/' + robot1['name']]

    # commenté pour slam
    """
    message_robot1 = (
        '{header: {frame_id: map}, pose: {pose: {position: {x: ' +
        robot1['x_pose'] + ', y: ' + robot1['y_pose'] +
        ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, }}'
    )
    
    initial_pose_robot1 = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-t', '3',
            '--qos-reliability', 'reliable',
            namespace_robot1 + ['/initialpose'],
            'geometry_msgs/PoseWithCovarianceStamped',
            message_robot1
        ],
        output='screen'
    )
    """

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

    post_spawn_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_action,
            on_exit=[ rviz_cmd],    #sans slam , metre [initial_pose_robot1, rviz_cmd],
        )
    )

    ld.add_action(post_spawn_event)

    return ld