#!/usr/bin/env python3

"""
BlueROV2 Stop All Thrusters Script

This script creates a ROS2 node that sends commands to stop all thrusters of a BlueROV2.
It sends a single stop command to each thruster and then exits.

Usage:
    ros2 run <package_name> bluerov2_stop_thrusters.py

To run with a custom model name:
    ros2 run <package_name> bluerov2_stop_thrusters.py --ros-args -p model_name:=my_rov

Parameters:
    model_name (string, default: 'bluerov2'): Name of the ROV model, used in topic names.

Note:
    This script assumes the BlueROV2 has 6 thrusters, numbered from 1 to 6.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from marsim_control_py.bluerov2_utils import publish_thruster_commands

class StopAllThrustersNode(Node):
    def __init__(self):
        super().__init__('stop_all_thrusters_node')
        
        self.declare_parameter('model_name', 'bluerov2')
        self.model_name = self.get_parameter('model_name').value
        
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/{self.model_name}/thruster{i}/cmd_thrust', 10)
            for i in range(1, 7)
        ]

    def stop_all_thrusters(self):
        stop_commands = [0.0] * 6
        publish_thruster_commands(self.thruster_pubs, stop_commands)
        self.get_logger().info('Stop commands sent to all thrusters')

def main(args=None):
    rclpy.init(args=args)
    node = StopAllThrustersNode()
    
    node.stop_all_thrusters()
    
    node.get_logger().info('Stop thruster operation completed. Exiting script.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()