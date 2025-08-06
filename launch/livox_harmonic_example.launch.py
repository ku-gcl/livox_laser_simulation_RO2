#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the package directory
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = FindPackageShare(package='ros2_livox_simulation').find('ros2_livox_simulation')
    
    # World file path
    world_file_path = 'worlds/empty.sdf'
    world_path = PathJoinSubstitution([pkg_share, world_file_path])
    
    # URDF/xacro file path  
    urdf_file_path = 'urdf/mid360_harmonic.xacro'
    urdf_path = PathJoinSubstitution([pkg_share, urdf_file_path])
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to world file'
    )
    
    # Gazebo Harmonic launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': LaunchConfiguration('world')}.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_path}],
    )
    
    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'livox_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
    ])