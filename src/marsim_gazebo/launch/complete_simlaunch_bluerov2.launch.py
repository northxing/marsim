#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration)

def generate_launch_description():
    # Declare parameters with descriptions
    spawn_delay_arg = DeclareLaunchArgument('spawn_delay', default_value='10.0', description='Delay in seconds before spawning the BlueROV2')
    bridge_delay_arg = DeclareLaunchArgument('bridge_delay', default_value='20.0', description='Delay in seconds before starting the ROS-GZ bridge')

    # Get the path to the launch files
    pkg_project_gazebo = FindPackageShare('marsim_gazebo')
    
    # Launch the empty world
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'empty_world.launch.py'])))

    # Launch BlueROV2 spawn after specified delay
    spawn_bluerov2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'spawn_bluerov2.launch.py'])))
    
    delayed_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[
            LogInfo(msg='Spawning BlueROV2...'),
            spawn_bluerov2_launch])

    # Launch bridge after specified delay
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'start_bridge_bluerov2.launch.py'])))
    
    delayed_bridge = TimerAction(
        period=LaunchConfiguration('bridge_delay'),
        actions=[
            LogInfo(msg='Starting ROS-GZ bridge...'),
            bridge_launch])

    # Return the launch description with proper sequencing
    return LaunchDescription([
        # Launch arguments
        spawn_delay_arg,
        bridge_delay_arg,
        # Launch sequence
        LogInfo(msg='Starting BlueROV2 simulation...'),
        empty_world_launch,
        delayed_spawn,
        delayed_bridge])

if __name__ == '__main__':
    generate_launch_description()