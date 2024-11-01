#!/usr/bin/env python3

"""
DepthSensorFromOdom ROS2 Node

This script implements a ROS2 node that subscribes to Odometry messages,
extracts the z-position (depth) information and its covariance, adds optional
Gaussian noise, and republishes it as a new Odometry message with only the
z-position and its covariance populated.

Key features:
1. Subscribes to a user-defined input topic for Odometry messages.
2. Extracts the z-position (depth) and its covariance from the input Odometry message.
3. Optionally adds Gaussian noise to the depth measurement.
4. Publishes a new Odometry message with only the z-position and its covariance to a user-defined output topic.
5. Uses ROS2 parameters for configuration (input and output topics, noise standard deviation).
6. Implements reliable QoS settings for message passing.

Usage:
    ros2 run <package_name> depth_sensor_from_odom

To run with custom parameters:
    ros2 run <package_name> depth_sensor_from_odom --ros-args -p input_topic:=/robot/odometry -p output_topic:=/bluerov2/depth_sensor/odom -p noise_stddev:=0.05

Parameters:
    input_topic (string, default: 'bluerov2/odometry'): The topic to subscribe to for input odometry messages.
    output_topic (string, default: 'bluerov2/depth_sensor/odom'): The topic to publish depth odometry messages to.
    noise_stddev (float, default: 0.0): Standard deviation of Gaussian noise to add to depth measurements. Set to 0.0 for no noise.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class DepthSensorFromOdom(Node):

    def __init__(self):
        super().__init__('depth_sensor_from_odom')

        # Declare parameters
        self.declare_parameter('input_topic', 'bluerov2/odometry')
        self.declare_parameter('output_topic', 'bluerov2/depth_sensor/odom')
        self.declare_parameter('noise_stddev', 0.01)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.noise_stddev = self.get_parameter('noise_stddev').value

        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.listener_callback,
            qos_profile)
        self.publisher = self.create_publisher(
            Odometry,
            self.output_topic,
            qos_profile)
        
        self.get_logger().info(f'DepthSensorFromOdom initialized with:')
        self.get_logger().info(f'  input_topic: {self.input_topic}')
        self.get_logger().info(f'  output_topic: {self.output_topic}')
        self.get_logger().info(f'  noise_stddev: {self.noise_stddev}')

    def add_noise(self, value):
        return value + np.random.normal(0, self.noise_stddev)

    def listener_callback(self, msg):
        depth_msg = Odometry()
        depth_msg.header = msg.header
        depth_msg.child_frame_id = msg.child_frame_id
        
        # Extract the z-position (depth) and add noise
        depth_msg.pose.pose.position.z = self.add_noise(msg.pose.pose.position.z)
        
        # Set all other position and orientation values to 0
        depth_msg.pose.pose.position.x = 0.0
        depth_msg.pose.pose.position.y = 0.0
        depth_msg.pose.pose.orientation.x = 0.0
        depth_msg.pose.pose.orientation.y = 0.0
        depth_msg.pose.pose.orientation.z = 0.0
        depth_msg.pose.pose.orientation.w = 1.0  # Set to 1 for a valid quaternion
        
        # Set all twist (velocity) values to 0
        depth_msg.twist.twist.linear.x = 0.0
        depth_msg.twist.twist.linear.y = 0.0
        depth_msg.twist.twist.linear.z = 0.0
        depth_msg.twist.twist.angular.x = 0.0
        depth_msg.twist.twist.angular.y = 0.0
        depth_msg.twist.twist.angular.z = 0.0
        
        # Initialize covariance matrices with zeros
        depth_msg.pose.covariance = [0.0] * 36
        depth_msg.twist.covariance = [0.0] * 36
        
        # Extract and set the z-position (depth) covariance
        # The z-position covariance is at index 14 in the 6x6 covariance matrix
        # (row 2, column 2 in 0-based indexing)
        depth_msg.pose.covariance[14] = msg.pose.covariance[14] + self.noise_stddev**2
        
        self.publisher.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_sensor_from_odom = DepthSensorFromOdom()
    rclpy.spin(depth_sensor_from_odom)
    depth_sensor_from_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()