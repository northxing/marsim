#!/usr/bin/env python3

"""
BlueROV2 PID-based Z Position Control ROS2 Node

This script implements a ROS2 node for controlling the depth (Z position) of a BlueROV2
underwater vehicle using a PID (Proportional-Integral-Derivative) controller.

Key features:
1. Subscribes to odometry data for current position and orientation.
2. Subscribes to a Z setpoint topic for desired depth commands.
3. Implements a PID control algorithm for depth regulation.
4. Publishes thrust commands to control the ROV's vertical motion.
5. Uses ROS2 parameters for easy configuration of control gains and topics.
6. Runs the control loop at a fixed frequency (10 Hz).

This node is essential for maintaining the desired depth of the BlueROV2 in underwater operations.

Usage:
    ros2 run <package_name> bluerov2_pid_pose_z_control

To run with custom parameters:
    ros2 run <package_name> bluerov2_pid_pose_z_control --ros-args -p model_name:=my_rov -p Kp:=15.0 -p Ki:=0.02 -p Kd:=0.5 -p desired_z:=-30.0

Parameters:
    model_name (string, default: 'bluerov2'): Name of the ROV model, used in topic names.
    ekf_topic (string, default: '/odometry/filtered'): Topic for receiving odometry data.
    z_setpoint_topic (string, default: '/z_setpoint'): Topic for receiving Z position setpoints.
    Kp (float, default: 10.0): Proportional gain for the PID controller.
    Ki (float, default: 0.01): Integral gain for the PID controller.
    Kd (float, default: 0.0): Derivative gain for the PID controller.
    desired_z (float, default: -45.0): Initial desired Z position (depth) of the ROV.

Example:
    The above command would run the node with the following configuration:
    - Using 'my_rov' as the model name for topic construction
    - Setting PID gains to Kp=15.0, Ki=0.02, Kd=0.5
    - Setting the initial desired depth to 30 meters

Note:
    - The control assumes that thrusters 5 and 6 are responsible for vertical motion.
    - Positive thrust pushes the ROV downwards.
    - Z position increases downwards (more negative values are higher in the water column).
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
from marsim_control_py.marsim_utils import odom_callback
from marsim_control_py.bluerov2_utils import publish_thruster_commands

class PoseZControlNode(Node):
    def __init__(self):
        super().__init__('z_position_control_node')
        
        # Declare parameters
        self.declare_parameter('model_name', 'bluerov2')
        self.declare_parameter('ekf_topic', '/bluerov2/odometry/filtered')
        self.declare_parameter('z_setpoint_topic', '/z_setpoint')
        self.declare_parameter('Kp', 10.0)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('desired_z', -45.0)

        # Get parameter values
        self.model_name = self.get_parameter('model_name').value
        self.ekf_topic = self.get_parameter('ekf_topic').value
        self.z_setpoint_topic = self.get_parameter('z_setpoint_topic').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.desired_z = self.get_parameter('desired_z').value

        # PID variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.ekf_topic}',
            self.process_odom,
            10)
        
        self.z_setpoint_sub = self.create_subscription(
            Float64,
            self.z_setpoint_topic,
            self.z_setpoint_callback,
            10)
        
        # Create publishers for each thruster
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/{self.model_name}/thruster{i}/cmd_thrust', 10)
            for i in range(1, 7)]
        
        # Timer for the control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

        self.get_logger().info(f'Node initialized with Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, desired_z={self.desired_z}')

    def process_odom(self, msg):
        state = odom_callback(msg)
        self.current_z = state[2]  # Z position is the third element in the state list
    
    def z_setpoint_callback(self, msg):
        self.desired_z = msg.data
        self.get_logger().info(f'Received new Z setpoint: {self.desired_z}')
    
    def control_loop(self):
        if not hasattr(self, 'current_z'):
            return  # Wait until we've received z position data

        current_time = self.get_clock().now()
        if self.previous_time is None:
            self.previous_time = current_time
            return  # Skip the first iteration to establish a time baseline

        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        error = self.desired_z - self.current_z
        
        # PID calculations
        self.integral += error * dt  
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Create thruster commands
        # Assuming thrusters 5 and 6 (indices 4 and 5) are responsible for vertical motion
        # Positive thrust for T5 and T6 pushes the ROV down
        thruster_commands = [None] * 6
        thruster_commands[4] = -output
        thruster_commands[5] = -output
        
        publish_thruster_commands(self.thruster_pubs, thruster_commands)

        self.previous_error = error
        self.previous_time = current_time

        # Log current state
        self.get_logger().info(f'Current Z: {self.current_z:.2f}, Desired Z: {self.desired_z:.2f}, '
                                f'Thruster command (T5/T6): {thruster_commands[4]:.2f} (+ve is downwards), '
                                f'dt: {dt:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseZControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()