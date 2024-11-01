from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('bluerov2_ctrl_serv_py')
    
    # Path to the parameter file
    config_file = os.path.join(
        package_share_dir,
        'config',
        'pid_disp_z_control_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='bluerov2_ctrl_serv_py',
            executable='pid_disp_z_control_action',
            name='pid_disp_z_control_action_server',
            parameters=[config_file],
            output='screen',
            #arguments=['--ros-args', '--log-level', 'debug']
        )
    ])