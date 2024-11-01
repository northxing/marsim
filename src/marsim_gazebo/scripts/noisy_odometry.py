#!/usr/bin/env python3

"""
NoisyOdometry ROS2 Node

This script implements a ROS2 node that subscribes to Odometry messages,
adds Gaussian noise to the measurements, and republishes them.

Key features:
1. Subscribes to a user-defined input topic for Odometry messages.
2. Adds configurable Gaussian noise to position, orientation, and velocities.
3. Publishes the noisy messages to a user-defined output topic.
4. Uses ROS2 parameters for configuration (topics and noise levels).
5. Implements reliable QoS settings for message passing.

This node is useful for simulating sensor noise or testing the robustness
of state estimation systems.

Usage:
    ros2 run <package_name> noisy_odometry

To run with custom parameters:
    ros2 run <package_name> noisy_odometry --ros-args -p input_topic:=/robot/odometry -p output_topic:=/robot/noisy_odometry -p pos_stddev:=0.02

Parameters:
    input_topic (string, default: 'odometry'): The topic to subscribe to for input odometry messages.
    output_topic (string, default: 'noisy_odometry'): The topic to publish noisy odometry messages to.
    pos_stddev (float, default: 0.01): Standard deviation for position noise.
    orient_stddev (float, default: 0.01): Standard deviation for orientation noise.
    lin_vel_stddev (float, default: 0.01): Standard deviation for linear velocity noise.
    ang_vel_stddev (float, default: 0.01): Standard deviation for angular velocity noise.

Example:
    The above command would run the node with the following configuration:
    - Subscribing to '/robot/odometry' for input
    - Publishing to '/robot/noisy_odometry' for output
    - Using a position noise standard deviation of 0.02
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class NoisyOdometry(Node):

    def __init__(self):
        super().__init__('noisy_odometry')

        # Declare parameters
        self.declare_parameter('input_topic', 'odometry')
        self.declare_parameter('output_topic', 'noisy_odometry')
        self.declare_parameter('pos_stddev', 0.01)
        self.declare_parameter('orient_stddev', 0.01)
        self.declare_parameter('lin_vel_stddev', 0.01)
        self.declare_parameter('ang_vel_stddev', 0.01)

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.pos_stddev = self.get_parameter('pos_stddev').value
        self.orient_stddev = self.get_parameter('orient_stddev').value
        self.lin_vel_stddev = self.get_parameter('lin_vel_stddev').value
        self.ang_vel_stddev = self.get_parameter('ang_vel_stddev').value

        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.listener_callback,
            qos_profile)
        self.publisher = self.create_publisher(
            Odometry,
            output_topic,
            qos_profile)
        
        self.get_logger().info(f'NoisyOdometry initialized with:')
        self.get_logger().info(f'  input_topic: {input_topic}')
        self.get_logger().info(f'  output_topic: {output_topic}')
        self.get_logger().info(f'  pos_stddev: {self.pos_stddev}')
        self.get_logger().info(f'  orient_stddev: {self.orient_stddev}')
        self.get_logger().info(f'  lin_vel_stddev: {self.lin_vel_stddev}')
        self.get_logger().info(f'  ang_vel_stddev: {self.ang_vel_stddev}')

    def add_noise(self, value, stddev):
        return value + np.random.normal(0, stddev)

    def listener_callback(self, msg):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id
        
        # Add noise to position
        noisy_msg.pose.pose.position.x = self.add_noise(msg.pose.pose.position.x, self.pos_stddev)
        noisy_msg.pose.pose.position.y = self.add_noise(msg.pose.pose.position.y, self.pos_stddev)
        noisy_msg.pose.pose.position.z = self.add_noise(msg.pose.pose.position.z, self.pos_stddev)
        
        # Add noise to orientation
        noisy_msg.pose.pose.orientation.x = self.add_noise(msg.pose.pose.orientation.x, self.orient_stddev)
        noisy_msg.pose.pose.orientation.y = self.add_noise(msg.pose.pose.orientation.y, self.orient_stddev)
        noisy_msg.pose.pose.orientation.z = self.add_noise(msg.pose.pose.orientation.z, self.orient_stddev)
        noisy_msg.pose.pose.orientation.w = self.add_noise(msg.pose.pose.orientation.w, self.orient_stddev)
        
        # Add noise to linear velocity
        noisy_msg.twist.twist.linear.x = self.add_noise(msg.twist.twist.linear.x, self.lin_vel_stddev)
        noisy_msg.twist.twist.linear.y = self.add_noise(msg.twist.twist.linear.y, self.lin_vel_stddev)
        noisy_msg.twist.twist.linear.z = self.add_noise(msg.twist.twist.linear.z, self.lin_vel_stddev)
        
        # Add noise to angular velocity
        noisy_msg.twist.twist.angular.x = self.add_noise(msg.twist.twist.angular.x, self.ang_vel_stddev)
        noisy_msg.twist.twist.angular.y = self.add_noise(msg.twist.twist.angular.y, self.ang_vel_stddev)
        noisy_msg.twist.twist.angular.z = self.add_noise(msg.twist.twist.angular.z, self.ang_vel_stddev)
        
        # Copy covariance matrices
        noisy_msg.pose.covariance = msg.pose.covariance
        noisy_msg.twist.covariance = msg.twist.covariance
        
        self.publisher.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    noisy_odometry = NoisyOdometry()
    rclpy.spin(noisy_odometry)
    noisy_odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()