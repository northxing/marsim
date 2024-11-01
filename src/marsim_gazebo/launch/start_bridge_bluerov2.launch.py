#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    set_logging_config = SetEnvironmentVariable(name='RCUTILS_LOGGING_USE_STDOUT', value='1')
    set_logging_format = SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='[{severity}] [{name}]: {message}')

    pkg_project_gazebo = FindPackageShare('marsim_gazebo')
    
    config_file_path = PathJoinSubstitution([pkg_project_gazebo, 'config', 'bluerov2_ros_gz_bridge.yaml'])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': config_file_path},
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    twist_stamper = Node(
        package='marsim_gazebo',
        executable='twist_stamper',
        name='bluerov2_twist_stamper',
        parameters=[{
            'frame_id': 'bluerov2/base_link/dvl_sensor',
            'input_topic': '/bluerov2/dvl/velocity_notStamped',
            'output_topic': '/bluerov2/dvl/velocity'
        }],
        output='screen'
    )

    # noisy_odometry adds noise to the odometry measurements that could be used to simulate position sensors
    noisy_odometry = Node(
        package='marsim_gazebo',  
        executable='noisy_odometry',
        name='bluerov2_noisy_odometry',
        parameters=[{
            'input_topic': 'bluerov2/odometry',
            'output_topic': 'bluerov2/noisy_odometry',
            'pos_stddev': 0.01,
            'orient_stddev': 0.01,
            'lin_vel_stddev': 0.01,
            'ang_vel_stddev': 0.01
        }],
        output='screen'
    )

    # The depth sensor is simulated using z pose odometry measurements + noise
    depth_sensor_from_odom = Node(
        package='marsim_gazebo',  
        executable='depth_sensor_from_odom',
        name='bluerov2_depth_sensor_from_odom',
        parameters=[{
            'input_topic': 'bluerov2/odometry',
            'output_topic': 'bluerov2/depth_sensor/odom',
            'noise_stddev': 0.01
        }],
        output='screen'
    )

    return LaunchDescription([
        set_logging_config,
        set_logging_format,
        bridge,
        twist_stamper,
        noisy_odometry,
        depth_sensor_from_odom])