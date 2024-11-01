#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bluerov2_action_interfaces.action import DispZControl
from rclpy.callback_groups import ReentrantCallbackGroup
import sys

class DispZControlClient(Node):
    def __init__(self):
        super().__init__('pid_disp_z_control_client')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action client
        self._action_client = ActionClient(
            self,
            DispZControl,
            'disp_z_control_action',
            callback_group=self.callback_group)
            
        # Add flags to track goal status
        self.goal_completed = False
        self.goal_result = None

    def send_goal(self, target_z):
        self.get_logger().info(f'Waiting for action server...')
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available after 10 seconds')
            rclpy.shutdown()
            return

        # Create and send goal
        goal_msg = DispZControl.Goal()
        goal_msg.target_z = float(target_z)

        self.get_logger().info(f'Sending goal: target_z = {target_z}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        
        # Log status and result
        self.get_logger().info(f'Status: {status}')
        self.get_logger().info(f'Final result received: final_z={result.final_z:.2f}, error={result.error:.2f}')
        
        self.goal_completed = True
        self.goal_result = result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: current_z={feedback.current_z:.2f}, '
            f'error={feedback.error:.2f}, thrust={feedback.thrust:.2f}')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print('Usage: ros2 run bluerov2_ctrl_serv_py pid_disp_z_control_client <target_z>')
        return

    try:
        target_z = float(sys.argv[1])
    except ValueError:
        print('Error: target_z must be a number')
        return

    action_client = DispZControlClient()
    
    try:
        # Send the goal
        action_client.send_goal(target_z)
        
        # Spin until the goal is complete
        while rclpy.ok() and not action_client.goal_completed:
            rclpy.spin_once(action_client)
            
    except KeyboardInterrupt:
        action_client.get_logger().info('Client stopped by user')
    except Exception as e:
        action_client.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        # Only call shutdown if rclpy was initialized
        if rclpy.ok():
            action_client.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()