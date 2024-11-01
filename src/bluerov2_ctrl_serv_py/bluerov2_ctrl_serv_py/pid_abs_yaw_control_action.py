#!/usr/bin/env python3

"""
BlueROV2 Absolute Yaw Control Action Server

This node provides an action server for controlling the absolute Yaw position of a BlueROV2
using PID control. It accepts absolute yaw position goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from bluerov2_action_interfaces.action import AbsYawControl
from marsim_control_py.marsim_utils import odom_callback, quaternion_to_euler
from marsim_control_py.bluerov2_utils import publish_thruster_commands
import numpy as np

class AbsYawControlAction(Node):
    def __init__(self):
        super().__init__('pid_abs_yaw_control_action')
        
        # Declare parameters
        self.declare_parameter('model_name', 'bluerov2')
        self.declare_parameter('ekf_topic', '/bluerov2/odometry/filtered')
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('position_tolerance', 0.01)  # rads
        self.declare_parameter('control_frequency', 10.0)  # Hz

        # Get parameter values
        self.model_name = self.get_parameter('model_name').value
        self.ekf_topic = self.get_parameter('ekf_topic').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            AbsYawControl,
            'abs_yaw_control_action',
            self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Create odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            self.ekf_topic,
            self.process_odom,
            10,
            callback_group=self.callback_group)
        
        # Create thruster publishers
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/{self.model_name}/thruster{i}/cmd_thrust', 10)
            for i in range(1, 7)]

        # Initialize control variables
        self.current_yaw = None
        self._goal_handle = None
        self.reset_pid()
        
        self.get_logger().info('Absolute Yaw Position Control Action Server has started')

    def reset_pid(self):
        """Reset PID controller variables"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def goal_callback(self, goal_request):
        """Accept or reject a new goal request"""
        self.get_logger().info('Received goal request')
        
        if self.current_yaw is None:
            self.get_logger().warn('No odometry data received yet')
            return GoalResponse.REJECT

        # Validate target yaw is within [-π, π]
        if abs(goal_request.target_yaw) > np.pi:
            self.get_logger().warn('Target yaw outside valid range [-π, π]')
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def process_odom(self, msg):
        """Process odometry data"""
        state = odom_callback(msg)
        q = np.zeros((1, 4))
        q[0, 0] = state[3]  # qx
        q[0, 1] = state[4]  # qy
        q[0, 2] = state[5]  # qz
        q[0, 3] = state[6]  # qw
        euler_angles = quaternion_to_euler(q)
        self.current_yaw = euler_angles[0, 2]  

    def stop_thrusters(self):
        """Stop all thrusters"""
        thruster_commands = [None] * 6
        thruster_commands[0] = 0.0  # Thruster 1
        thruster_commands[1] = 0.0  # Thruster 2
        thruster_commands[2] = 0.0  # Thruster 3
        thruster_commands[3] = 0.0  # Thruster 4
        publish_thruster_commands(self.thruster_pubs, thruster_commands)

    def execute_callback(self, goal_handle):
        """Execute the control action"""
        self.get_logger().info('Executing goal...')
        
        # Reset PID controller
        self.reset_pid()
        
        # Get requested displacement
        target_yaw = goal_handle.request.target_yaw
        
        feedback_msg = AbsYawControl.Feedback()
        result = AbsYawControl.Result()

        try:
            # Create ROS rate object
            rate = self.create_rate(self.control_frequency)
            
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_thrusters()
                    result.final_yaw = self.current_yaw
                    result.error = target_yaw - self.current_yaw
                    self.get_logger().info('Goal canceled')
                    return result

                current_time = self.get_clock().now()
                if self.previous_time is None:
                    self.previous_time = current_time
                    rate.sleep()
                    continue

                dt = (current_time - self.previous_time).nanoseconds / 1e9

                # Calculate desired position
                desired_position = target_yaw

                # Calculate error (desired - current)
                error = desired_position - self.current_yaw
                
                # PID calculations
                self.integral += error * dt
                derivative = (error - self.previous_error) / dt if dt > 0 else 0
                
                output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
                
                # Create and publish thruster commands
                thruster_commands = [None] * 6
                thruster_commands[0] = -output
                thruster_commands[1] = output
                thruster_commands[2] = output
                thruster_commands[3] = -output
                
                publish_thruster_commands(self.thruster_pubs, thruster_commands)

                # Update state variables
                self.previous_error = error
                self.previous_time = current_time

                # Publish feedback
                feedback_msg.current_yaw = self.current_yaw
                feedback_msg.error = error
                feedback_msg.thrust_t1 = thruster_commands[0]
                feedback_msg.thrust_t2 = thruster_commands[1]
                feedback_msg.thrust_t3 = thruster_commands[2]
                feedback_msg.thrust_t4 = thruster_commands[3]
                goal_handle.publish_feedback(feedback_msg)

                # Log status
                self.get_logger().info(
                    f'Current Yaw: {self.current_yaw:.2f}, Target Yaw: {target_yaw:.2f}, Error: {error:.2f}, '
                    f'Thrust T1: {thruster_commands[0]:.2f}, Thrust T2: {thruster_commands[1]:.2f} '
                    f'Thrust T3: {thruster_commands[2]:.2f}, Thrust T4: {thruster_commands[3]:.2f} '
                )

                # Check if we've reached the target
                if abs(error) < self.position_tolerance:
                    break

                rate.sleep()

        except Exception as e:
            self.get_logger().error(f'Error in execute_callback: {str(e)}')
            goal_handle.abort()
            return result

        # Goal succeeded
        goal_handle.succeed()
        
        # Stop thrusters when done
        self.stop_thrusters()
        
        # Set result
        result.final_yaw = self.current_yaw
        result.error = error
        
        self.get_logger().info('Goal succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the action server with a MultiThreadedExecutor
    node = AbsYawControlAction()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_thrusters()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()