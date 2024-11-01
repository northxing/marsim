#!/usr/bin/env python3

"""
TwistStamper ROS2 Node

This script implements a ROS2 node that subscribes to unstamped TwistWithCovariance
messages and republishes them as stamped TwistWithCovarianceStamped messages.

Key features:
1. Subscribes to a user-defined input topic for TwistWithCovariance messages.
2. Subscribes to the /clock topic to get the current simulation time.
3. Adds a header with a timestamp and frame_id to each incoming twist message.
4. Publishes the stamped messages to a user-defined output topic.
5. Uses ROS2 parameters for configuration (frame_id, input_topic, output_topic).
6. Implements reliable QoS settings for message passing.

This node is useful in robotics simulations where unstamped twist messages need
to be converted to stamped messages with the correct simulation time.

Usage:
    ros2 run <package_name> twist_stamper

To run with custom parameters:
    ros2 run <package_name> twist_stamper --ros-args -p frame_id:=odom -p input_topic:=/robot/twist_unstamped -p output_topic:=/robot/twist_stamped

Parameters:
    frame_id (string, default: 'base_link'): The frame ID to use in the stamped message header.
    input_topic (string, default: 'twist_unstamped'): The topic to subscribe to for unstamped twist messages.
    output_topic (string, default: 'twist_stamped'): The topic to publish stamped twist messages to.

Example:
    The above command would run the node with the following configuration:
    - frame_id set to 'odom'
    - Subscribing to '/robot/twist_unstamped' for input
    - Publishing to '/robot/twist_stamped' for output
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovariance, TwistWithCovarianceStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

class TwistStamper(Node):

    def __init__(self):
        super().__init__('twist_stamper')

        # Declare parameters
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('input_topic', 'twist_unstamped')
        self.declare_parameter('output_topic', 'twist_stamped')

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to the clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.current_time = None

        self.subscription = self.create_subscription(
            TwistWithCovariance,
            input_topic,
            self.listener_callback,
            qos_profile)
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            output_topic,
            qos_profile)
        
        self.get_logger().info(f'TwistStamper initialized with:')
        self.get_logger().info(f'  frame_id: {self.frame_id}')
        self.get_logger().info(f'  input_topic: {input_topic}')
        self.get_logger().info(f'  output_topic: {output_topic}')
        self.get_logger().info(f'  Using time from /clock topic')

    def clock_callback(self, msg):
        self.current_time = msg.clock

    def listener_callback(self, msg):
        if self.current_time is None:
            self.get_logger().warn('No clock message received yet. Skipping this message.')
            return

        stamped_msg = TwistWithCovarianceStamped()
        stamped_msg.header = Header()
        
        # Use time from /clock topic
        stamped_msg.header.stamp = self.current_time
        
        stamped_msg.header.frame_id = self.frame_id
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)
        #self.get_logger().info(f'Published stamped twist message with frame_id: {self.frame_id} and time: {self.current_time.sec}.{self.current_time.nanosec:09d}')

def main(args=None):
    rclpy.init(args=args)
    twist_stamper = TwistStamper()
    rclpy.spin(twist_stamper)
    twist_stamper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()