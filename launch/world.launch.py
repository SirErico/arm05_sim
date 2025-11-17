#!/usr/bin/env python3

import os

import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np


def _resolve_world_name(world_path: str) -> str:
    try:
        tree = ET.parse(world_path)
    except (ET.ParseError, FileNotFoundError):
        return 'default'

    root = tree.getroot()
    world_element = root.find('world')
    if world_element is not None:
        return world_element.get('name', 'default') or 'default'

    # Fall back to searching in case of nested structure
    for candidate in root.findall('.//world'):
        name = candidate.get('name')
        if name:
            return name

    return 'default'


def generate_launch_description():
    turtlebot3_share_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_launch_dir = os.path.join(turtlebot3_share_dir, 'launch')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
    )
    arm05_share_dir = get_package_share_directory('arm05_sim')

    arm05_models_path = os.path.join(arm05_share_dir, 'models')
    turtlebot3_models_path = os.path.join(turtlebot3_share_dir, 'models')

    existing_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_paths = [arm05_models_path, turtlebot3_models_path]
    if existing_gazebo_path:
        gazebo_paths.append(existing_gazebo_path)
    gazebo_model_path = os.pathsep.join(gazebo_paths)
    default_tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model', default=default_tb3_model)

    resource_env_actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    aruco_seed = LaunchConfiguration('aruco_seed', default='')
    # Launch argument mirrors existing TURTLEBOT3_MODEL env or defaults to waffle

    aruco_seed_launch_arg = DeclareLaunchArgument(
        'aruco_seed',
        default_value=str(np.random.randint(0, 9999999)),
        description="Seed for aruco spawner"
    )
    turtlebot3_model_launch_arg = DeclareLaunchArgument(
        'turtlebot3_model',
        default_value=default_tb3_model,
        description='TurtleBot3 model to spawn (burger, waffle, waffle_pi)'
    )

    world = os.path.join(arm05_share_dir, 'worlds', 'arm_house.world')
    world_name = _resolve_world_name(world)
    map_file = os.path.join(arm05_share_dir, 'map', 'map.yaml')
    params_file = os.path.join(arm05_share_dir, 'param', 'waffle.yaml')

    simulation_actions = []

    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Unable to locate simulation backend. Install 'ros_gz_sim' to run the arm05_sim world."
        ) from exc

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_paths = [arm05_models_path, turtlebot3_models_path]
    if existing_resource_path:
        gz_resource_paths.append(existing_resource_path)
    gz_resource_path = os.pathsep.join(gz_resource_paths)

    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_resource_paths = [arm05_models_path, turtlebot3_models_path]
    if existing_ign_path:
        ign_resource_paths.append(existing_ign_path)
    ign_resource_path = os.pathsep.join(ign_resource_paths)

    gz_server_args = f'-r -s -v2 {world}'
    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_server_args,
            'on_exit_shutdown': 'true',
        }.items()
    )

    gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true',
        }.items()
    )

    simulation_actions.extend([gz_server_cmd, gz_client_cmd])
    resource_env_actions.append(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path))
    resource_env_actions.append(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path))

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # don't need nav2 launch, running it separately
    # navigation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_launch_dir, 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'map': map_file,
    #         'params_file': params_file,
    #         'use_sim_time': use_sim_time,
    #         'autostart': 'true'
    #     }.items()
    # )


    spawn_aruco_cubes = Node(
        package='arm05_sim',
        executable='spawn_aruco.py',
        namespace='spawn_cubes',
        output='screen',
        arguments=[
            '--seed', aruco_seed,
        ]
    )

    delayed_spawn_aruco = TimerAction(
        period=5.0,
        actions=[spawn_aruco_cubes]
    )
    
    aruco_detection = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
        parameters=[{
            'marker_size': 0.09,
            'aruco_dictionary_id': 'DICT_ARUCO_ORIGINAL',
            'camera_frame': 'camera_rgb_optical_frame',
            'reference_frame': 'map',
            'use_sim_time': use_sim_time,
        }]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(aruco_seed_launch_arg)
    ld.add_action(turtlebot3_model_launch_arg)
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model))
    for env_action in resource_env_actions:
        ld.add_action(env_action)
    for action in simulation_actions:
        ld.add_action(action)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(delayed_spawn_aruco)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'turtlebot3_model': turtlebot3_model,
            }.items()
        )
    )
    # ld.add_action(navigation_cmd)
    ld.add_action(aruco_detection)


    return ld
