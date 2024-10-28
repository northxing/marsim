#!/usr/bin/env python3

"""
BlueROV2 PID-based XY Position and Yaw Control ROS2 Node

This script implements a ROS2 node for controlling the horizontal position (X,Y) and yaw orientation 
of a BlueROV2 underwater vehicle using PID (Proportional-Integral-Derivative) controllers.

Key features:
1. Executes a three-phase movement sequence: rotation to travel heading, position movement, final rotation
2. Uses separate PID controllers for position and yaw control
3. Subscribes to odometry data for current position and orientation
4. Implements settling time and error thresholds for precise control
5. Includes safety stop sequences between movement phases
6. Uses ROS2 parameters for easy configuration of control gains and movement targets
7. Runs the control loop at a fixed frequency (10 Hz)

This node is essential for precise horizontal navigation and orientation control of the BlueROV2.

Usage:
    ros2 run <package_name> bluerov2_pid_pose_xyyaw_reqcontrol.py

To run with custom parameters:
    ros2 run <package_name> bluerov2_pid_pose_xyyaw_reqcontrol.py --ros-args -p model_name:=my_rov -p pos_Kp:=5.0 -p pos_Ki:=0.005 -p yaw_Kp:=1.0 -p req_disp_x:=3.0 -p req_disp_y:=5.0

Parameters:
    model_name (string, default: 'bluerov2'): Name of the ROV model, used in topic names
    ekf_topic (string, default: '/bluerov2/odometry/filtered'): Topic for receiving odometry data
    pos_Kp (float, default: 5.0): Position controller proportional gain
    pos_Ki (float, default: 0.005): Position controller integral gain
    pos_Kd (float, default: 0.0): Position controller derivative gain
    yaw_Kp (float, default: 1.0): Yaw controller proportional gain
    yaw_Ki (float, default: 0.001): Yaw controller integral gain
    yaw_Kd (float, default: 0.0): Yaw controller derivative gain
    req_disp_x (float, default: 3.0): Required displacement in X direction
    req_disp_y (float, default: 5.0): Required displacement in Y direction
    req_abs_yaw (float, default: -1.0): Required absolute yaw angle in radians
    pos_error_threshold (float, default: 0.1): Position error threshold for completion
    yaw_error_threshold (float, default: 0.02): Yaw error threshold for completion
    pos_settling_time (float, default: 5.0): Required time within position threshold
    yaw_settling_time (float, default: 5.0): Required time within yaw threshold

Example:
    The above command would run the node with the following configuration:
    - Using 'my_rov' as the model name for topic construction
    - Setting position PID gains to Kp=5.0, Ki=0.005
    - Setting target displacement to X=3.0m, Y=5.0m
    - Using default values for other parameters

Note:
    - The control assumes thrusters 1-4 are responsible for horizontal motion and yaw
    - Position control uses a calculated heading based on target X,Y coordinates
    - Each movement phase includes a 2-second stop sequence for safety
    - The controller uses displacement from initial position, not absolute coordinates
    - Yaw angles are in radians, with positive rotation being counter-clockwise
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import math
from marsim_control.marsim_utils import odom_callback, quaternion_to_euler
from marsim_control.bluerov2_utils import publish_thruster_commands

class PoseXYYawControlNode(Node):
    def __init__(self):
        super().__init__('xyyaw_position_control_node')

        # Declare parameters
        self.declare_parameter('model_name', 'bluerov2')
        self.declare_parameter('ekf_topic', '/bluerov2/odometry/filtered')
        self.declare_parameter('pos_Kp', 5.0)
        self.declare_parameter('pos_Ki', 0.005)
        self.declare_parameter('pos_Kd', 0.0)
        self.declare_parameter('yaw_Kp', 1.0)
        self.declare_parameter('yaw_Ki', 0.001)
        self.declare_parameter('yaw_Kd', 0.00)
        self.declare_parameter('req_disp_x', 3.0)
        self.declare_parameter('req_disp_y', 5.0)
        self.declare_parameter('req_abs_yaw', -1.0)
        self.declare_parameter('pos_error_threshold', 0.1)  
        self.declare_parameter('yaw_error_threshold', 0.02)  
        self.declare_parameter('pos_settling_time', 5.0)   
        self.declare_parameter('yaw_settling_time', 5.0) 

        # Get parameter values
        self.model_name = self.get_parameter('model_name').value
        self.ekf_topic = self.get_parameter('ekf_topic').value
        self.pos_Kp = self.get_parameter('pos_Kp').value
        self.pos_Ki = self.get_parameter('pos_Ki').value
        self.pos_Kd = self.get_parameter('pos_Kd').value
        self.yaw_Kp = self.get_parameter('yaw_Kp').value
        self.yaw_Ki = self.get_parameter('yaw_Ki').value
        self.yaw_Kd = self.get_parameter('yaw_Kd').value
        self.req_disp_x = self.get_parameter('req_disp_x').value
        self.req_disp_y = self.get_parameter('req_disp_y').value
        self.req_abs_yaw = self.get_parameter('req_abs_yaw').value
        self.pos_error_threshold = self.get_parameter('pos_error_threshold').value
        self.yaw_error_threshold = self.get_parameter('yaw_error_threshold').value
        self.pos_settling_time = self.get_parameter('pos_settling_time').value
        self.yaw_settling_time = self.get_parameter('yaw_settling_time').value

        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry, f'{self.ekf_topic}', self.process_odom, 10)

        # Create publishers for each thruster
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/{self.model_name}/thruster{i}/cmd_thrust', 10)
            for i in range(1, 7)]

        # Read the current x, y and yaw positions and set it to be the initial x, y and yaw positions
        self.initial_x = self.read_initial_position('x')
        self.initial_y = self.read_initial_position('y')
        self.initial_yaw = self.read_initial_position('yaw')

        # Calculate the desired yaw heading
        self.req_yaw_heading = np.arctan2(self.req_disp_y, self.req_disp_x) 

        # Print to the screen important parameters
        self.get_logger().info(f'Node initialized with pos_Kp={self.pos_Kp}, pos_Ki={self.pos_Ki}, pos_Kd={self.pos_Kd}, \n'
                                f'yaw_Kp={self.yaw_Kp}, yaw_Ki={self.yaw_Ki}, yaw_Kd={self.yaw_Kd}, \n' 
                                f'req_disp_x={self.req_disp_x:.2f}, initial_x={self.initial_x:.2f}, \n'
                                f'req_disp_y={self.req_disp_y:.2f}, initial_y={self.initial_y:.2f}, \n'
                                f'req_abs_yaw={self.req_abs_yaw:.2f}, initial_yaw={self.initial_yaw:.2f}, req_yaw_heading={self.req_yaw_heading:.2f}')

        # Start the main control loop
        self.init_control_vars()
        self.start_movement_sequence()
        self.timer = self.create_timer(0.1, self.main_control_loop)  # 10 Hz control loop

    def process_odom(self, msg):
        state = odom_callback(msg)
        self.current_x = state[0]  # X position is the first element in the state list
        self.current_y = state[1]  # Y position is the second element in the state list
        q = np.zeros((1, 4))
        q[0, 0] = state[3]  # qx
        q[0, 1] = state[4]  # qy
        q[0, 2] = state[5]  # qz
        q[0, 3] = state[6]  # qw
        euler_angles = quaternion_to_euler(q)
        self.current_yaw = euler_angles[0, 2]    

    def read_initial_position(self, position_type):
        position_map = {
            'x': 'current_x',
            'y': 'current_y',
            'yaw': 'current_yaw'}
        if position_type not in position_map:
            raise ValueError(f"Invalid position_type. Must be one of {list(position_map.keys())}")
        attr_name = position_map[position_type]
        self.get_logger().info(f'Waiting for initial {position_type.upper()} position...')
        initial_value = None
        while rclpy.ok() and initial_value is None:
            rclpy.spin_once(self, timeout_sec=1.0)
            if hasattr(self, attr_name):
                initial_value = getattr(self, attr_name)
        self.get_logger().info(f'Initial {position_type.upper()} position received: {initial_value:.2f}')
        return initial_value

    def init_control_vars(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        self.time_within_threshold = None
        self.stop_start_time = None
        self.stopping = False   
        self.movement_completed = False

    def main_control_loop(self):
        if self.control_state == 'IDLE':
            self.get_logger().info('Movement sequence completed, shutting down node.')
            self.timer.cancel()
            self.movement_completed = True
            return
            
        elif self.control_state == 'ROTATING_TO_TRAVEL':
            if not hasattr(self, 'phase_timer'):
                # Start the yaw control for travel orientation
                self.init_control_vars()
                self.phase_timer = self.create_timer(0.1, lambda: self.yaw_control_loop(self.req_yaw_heading))
            
            # Check if the phase timer has been cancelled (indicating completion)
            if hasattr(self, 'phase_timer') and self.phase_timer.is_canceled():
                self.phase_timer.destroy()
                delattr(self, 'phase_timer')
                self.control_state = 'MOVING_TO_POSITION'
                self.get_logger().info('Completed rotation to travel heading, starting position control')
                
        elif self.control_state == 'MOVING_TO_POSITION':
            if not hasattr(self, 'phase_timer'):
                # Start the position control
                target_disp = math.sqrt(self.req_disp_x**2 + self.req_disp_y**2)
                self.init_control_vars()
                self.phase_timer = self.create_timer(0.1, lambda: self.pos_control_loop(target_disp))
            
            if hasattr(self, 'phase_timer') and self.phase_timer.is_canceled():
                self.phase_timer.destroy()
                delattr(self, 'phase_timer')
                self.control_state = 'ROTATING_TO_FINAL'
                self.get_logger().info('Reached target position, starting final rotation')
                
        elif self.control_state == 'ROTATING_TO_FINAL':
            if not hasattr(self, 'phase_timer'):
                # Start the yaw control for travel orientation
                self.init_control_vars()
                self.phase_timer = self.create_timer(0.1, lambda: self.yaw_control_loop(self.req_abs_yaw))
            
            # Check if the phase timer has been cancelled (indicating completion)
            if hasattr(self, 'phase_timer') and self.phase_timer.is_canceled():
                self.phase_timer.destroy()
                delattr(self, 'phase_timer')
                self.control_state = 'IDLE'
                self.get_logger().info('Completed rotation to travel heading, going to idle')

    def start_movement_sequence(self):
        """Initiate the full movement sequence"""
        self.get_logger().info('Starting movement sequence')
        self.control_state = 'ROTATING_TO_TRAVEL'

    def pos_control_loop(self, target_disp):
        current_time = self.get_clock().now()
        if self.previous_time is None:
            self.previous_time = current_time
            return  # Skip the first iteration to establish a time baseline

        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Check if in stopping phase
        if hasattr(self, 'stopping') and self.stopping:
            # In stopping phase - continue sending zero commands
            thruster_commands = [None] * 6
            thruster_commands[0] = 0.0
            thruster_commands[1] = 0.0
            thruster_commands[2] = 0.0
            thruster_commands[3] = 0.0
            publish_thruster_commands(self.thruster_pubs, thruster_commands)
        
            # Check if we've been stopping for 2 seconds
            if (current_time - self.stop_start_time).nanoseconds / 1e9 >= 2.0:
                self.get_logger().info('Stop commands sent for 2 seconds. Exiting.')
                self.phase_timer.cancel()
                return
            return
        
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        current_disp = math.sqrt(dx**2 + dy**2)
        error = target_disp - current_disp

        # Check if target is reached
        if abs(error) < self.pos_error_threshold:
            if self.time_within_threshold is None:
                self.time_within_threshold = current_time
            elif (current_time - self.time_within_threshold).nanoseconds / 1e9 >= self.pos_settling_time:
                self.get_logger().info('Target position reached and settled! Starting 2-second stop sequence.')
                # Initialize stopping sequence
                self.stopping = True
                self.stop_start_time = current_time
                return
        else:
            self.time_within_threshold = None

        # PID calculations
        self.integral += error * dt  
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = self.pos_Kp * error + self.pos_Ki * self.integral + self.pos_Kd * derivative

        # Create thruster commands
        thruster_commands = [None] * 6
        thruster_commands[0] = -output
        thruster_commands[1] = -output
        thruster_commands[2] = output
        thruster_commands[3] = output

        publish_thruster_commands(self.thruster_pubs, thruster_commands)

        self.previous_error = error
        self.previous_time = current_time

        # Log current state
        self.get_logger().info(f'Current displacement: {current_disp:.2f}, Target displacement: {target_disp:.2f}, '
                                f'Curent position: ({self.current_x:.2f},{self.current_y:.2f}), Initial position: ({self.initial_x:.2f},{self.initial_y:.2f}) '
                                f'Thruster command (T1): {thruster_commands[0]:.2f}, '
                                f'Thruster command (T2): {thruster_commands[1]:.2f}, '
                                f'Thruster command (T3): {thruster_commands[2]:.2f}, '
                                f'Thruster command (T4): {thruster_commands[3]:.2f}, '
                                f'dt: {dt:.4f}')

    def yaw_control_loop(self, target_abs_yaw):
        current_time = self.get_clock().now()
        if self.previous_time is None:
            self.previous_time = current_time
            return  # Skip the first iteration to establish a time baseline

        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Check if in stopping phase
        if hasattr(self, 'stopping') and self.stopping:
            # In stopping phase - continue sending zero commands
            thruster_commands = [None] * 6
            thruster_commands[0] = 0.0
            thruster_commands[1] = 0.0
            thruster_commands[2] = 0.0
            thruster_commands[3] = 0.0
            publish_thruster_commands(self.thruster_pubs, thruster_commands)
        
            # Check if we've been stopping for 2 seconds
            if (current_time - self.stop_start_time).nanoseconds / 1e9 >= 2.0:
                self.get_logger().info('Stop commands sent for 2 seconds. Exiting.')
                self.phase_timer.cancel()
                return
            return

        error = target_abs_yaw - self.current_yaw

        # Check if target is reached
        if abs(error) < self.yaw_error_threshold:
            if self.time_within_threshold is None:
                self.time_within_threshold = current_time
            elif (current_time - self.time_within_threshold).nanoseconds / 1e9 >= self.yaw_settling_time:
                self.get_logger().info('Target yaw reached and settled! Starting 2-second stop sequence.')
                # Initialize stopping sequence
                self.stopping = True
                self.stop_start_time = current_time
                return
        else:
            self.time_within_threshold = None

        # PID calculations
        self.integral += error * dt  
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = self.yaw_Kp * error + self.yaw_Ki * self.integral + self.yaw_Kd * derivative

        # Create thruster commands
        thruster_commands = [None] * 6
        thruster_commands[0] = -output
        thruster_commands[1] = output
        thruster_commands[2] = output
        thruster_commands[3] = -output

        publish_thruster_commands(self.thruster_pubs, thruster_commands)

        self.previous_error = error
        self.previous_time = current_time

        # Log current state
        self.get_logger().info(f'Current Yaw: {self.current_yaw:.2f}, Target absolute Yaw: {target_abs_yaw:.2f}, Initial Yaw: {self.initial_yaw:.2f} '
                                f'Thruster command (T1): {thruster_commands[0]:.2f}, '
                                f'Thruster command (T2): {thruster_commands[1]:.2f}, '
                                f'Thruster command (T3): {thruster_commands[2]:.2f}, '
                                f'Thruster command (T4): {thruster_commands[3]:.2f}, '
                                f'dt: {dt:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseXYYawControlNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.movement_completed:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()