#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package containing the Gazebo launch files
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    pkg_project_gazebo = FindPackageShare('marsim_gazebo').find('marsim_gazebo')

    # Construct the gz_args
    gz_args = [
        PathJoinSubstitution([pkg_project_gazebo, 'worlds', 'empty.world']),
        TextSubstitution(text=' -v 4')  # Add verbosity flag to Gazebo arguments
    ]

    # Launch the Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return LaunchDescription([
        gz_sim
    ])