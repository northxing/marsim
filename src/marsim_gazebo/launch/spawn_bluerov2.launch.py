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
    pkg_project_gazebo_str = get_package_share_directory('marsim_gazebo')
    pkg_robot_localization = get_package_share_directory('robot_localization')

    sdf_file_arg = DeclareLaunchArgument(
        'sdf_file_arg',
        default_value=PathJoinSubstitution([pkg_project_gazebo, 'models', 'bluerov2', 'model.sdf']),
        description='Path to the BlueROV2 SDF model file'
    )
    
    # Returns the exact full path of the file in a string format
    sdf_file = os.path.join(pkg_project_gazebo_str, 'models', 'bluerov2', 'model.sdf')

    # Read in the sdf file as a string
    with open(sdf_file, 'r') as infp: robot_desc = infp.read()

    robot_name_arg = DeclareLaunchArgument('robot_name_arg', default_value='bluerov2')
    x_pos_arg = DeclareLaunchArgument('x_pos_arg', default_value='0.0')
    y_pos_arg = DeclareLaunchArgument('y_pos_arg', default_value='0.0')
    z_pos_arg = DeclareLaunchArgument('z_pos_arg', default_value='-98.0')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name_arg'),
            '-x', LaunchConfiguration('x_pos_arg'),
            '-y', LaunchConfiguration('y_pos_arg'),
            '-z', LaunchConfiguration('z_pos_arg'),
            '-file', LaunchConfiguration('sdf_file_arg'),
            '--ros-args', '--log-level', 'info'],
        output='screen')

    # Static transform from world to robot's odom
    static_transform_world_to_robot_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_bluerov2_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'bluerov2/odom'])

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='bluerov2_robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'publish_frequency': 0.0,},  # Disable TF publishing
            {'use_sim_time': True},
            {'frame_prefix': 'bluerov2/'}],  # Add this if you want to prefix all frames
        remappings=[('/joint_states', '/bluerov2/joint_states'),  # Remap joint states
                    ('/robot_description', '/bluerov2/robot_description')],  # Remap robot description topic
        arguments=['--ros-args', '--log-level', 'info'])

    # Define the path to the EKF configuration file
    ekf_config_path = os.path.join(pkg_project_gazebo_str, 'config', 'bluerov2_ekf_config.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',         # this name cannot be changed, otherwise robot_localization will not work
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': True},
            {'tf_prefix': 'bluerov2'},
            {'initial_state': [
                LaunchConfiguration('x_pos_arg'),
                LaunchConfiguration('y_pos_arg'),
                LaunchConfiguration('z_pos_arg'),
                0.0, 0.0, 0.0,      # roll, pitch, yaw
                0.0, 0.0, 0.0,      # vx, vy, vz
                0.0, 0.0, 0.0,      # vroll, vpitch, vyaw
                0.0, 0.0, 0.0]}],   # ax, ay, az      
        remappings=[('/odometry/filtered', '/bluerov2/odometry/filtered')],
        arguments=['--ros-args', '--log-level', 'info'])

    return LaunchDescription([
        set_logging_config,
        set_logging_format,
        sdf_file_arg,
        robot_name_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        spawn_entity,
        static_transform_world_to_robot_odom,
        robot_state_publisher,
        robot_localization_node
        ])
