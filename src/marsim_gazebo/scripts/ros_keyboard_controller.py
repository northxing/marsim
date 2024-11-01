#!/usr/bin/env python3


"""
ROS2 Keyboard Controller for ROV (Remotely Operated Vehicle)

This script implements a keyboard-controlled interface for operating an ROV (Remotely Operated Vehicle)
in a ROS2 environment. It allows users to control the ROV's movement using simple keyboard commands,
which are translated into thruster commands and published to ROS2 topics.

Key features:
1. Keyboard-based control for intuitive ROV operation.
2. Controls 6 thrusters for various movements (forward, backward, up, down, rotate CW, rotate CCW).
3. Publishes thruster commands to ROS2 topics.
4. Supports custom model names for flexibility in multi-robot setups.
5. Provides real-time feedback on the current movement.
6. Implements a clean exit mechanism.

Usage:
    python ros_keyboard_controller.py <model_name>

Arguments:
    model_name (string): The name of the ROV model in the ROS2 environment.

Controls:
    w: Move forward
    x: Move backward
    a: Rotate counter-clockwise
    d: Rotate clockwise
    q: Move up
    e: Move down
    s: Stop all thrusters
    r: Quit the controller

Example:
    python ros_keyboard_controller.py my_rov

    This would start the keyboard controller for an ROV model named "my_rov".
    The script will then publish commands to topics like:
    /my_rov/thruster1/cmd_thrust, /my_rov/thruster2/cmd_thrust, etc.

Note:
    This script requires ROS2 to be properly set up in your environment.
    It uses the 'ros2 topic pub' command to publish thruster commands.

Dependencies:
    - ROS2
    - Python 3
    - sys, subprocess, tty, termios modules
"""

import sys
import subprocess
import tty
import termios

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def execute_ros2_command(model_name, command):
    thrusters = {
        'forward': [('-5.0', '1'), ('-5.0', '2'), ('5.0', '3'), ('5.0', '4')],
        'backward': [('5.0', '1'), ('5.0', '2'), ('-5.0', '3'), ('-5.0', '4')],
        'up': [('-5.0', '5'), ('-5.0', '6')],
        'down': [('5.0', '5'), ('5.0', '6')],
        'ccw': [('-5.0', '1'), ('5.0', '2'), ('5.0', '3'), ('-5.0', '4')],
        'cw': [('5.0', '1'), ('-5.0', '2'), ('-5.0', '3'), ('5.0', '4')],
        'stop': [('0.0', '1'), ('0.0', '2'), ('0.0', '3'), ('0.0', '4'), ('0.0', '5'), ('0.0', '6')]
    }
    
    commands = []
    for thrust, thruster_num in thrusters[command]:
        commands.append(f"ros2 topic pub --once /{model_name}/thruster{thruster_num}/cmd_thrust std_msgs/msg/Float64 '{{data: {thrust}}}'")
    
    combined_command = " & ".join(commands)
    
    for _ in range(3):
        subprocess.run(combined_command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def print_instructions():
    print("\nROV Keyboard Controller")
    print("------------------------")
    print("w: Forward, x: Backward, a: CCW, d: CW")
    print("q: Up, e: Down, s: Stop, r: Quit")
    print("Press any key to control (no Enter required)")

def main():
    if len(sys.argv) != 2:
        print("Usage: python rov_controller.py <model_name>")
        sys.exit(1)

    model_name = sys.argv[1]
    print_instructions()

    while True:
        command = getch().lower()
        if command == 'w':
            execute_ros2_command(model_name, 'forward')
            print("\rMoving forward ", end="", flush=True)
        elif command == 'x':
            execute_ros2_command(model_name, 'backward')
            print("\rMoving backward", end="", flush=True)
        elif command == 'a':
            execute_ros2_command(model_name, 'ccw')
            print("\rRotating CCW  ", end="", flush=True)
        elif command == 'd':
            execute_ros2_command(model_name, 'cw')
            print("\rRotating CW   ", end="", flush=True)
        elif command == 'q':
            execute_ros2_command(model_name, 'up')
            print("\rMoving up     ", end="", flush=True)
        elif command == 'e':
            execute_ros2_command(model_name, 'down')
            print("\rMoving down   ", end="", flush=True)
        elif command == 's':
            execute_ros2_command(model_name, 'stop')
            print("\rStopping      ", end="", flush=True)
        elif command == 'r':
            print("\nExiting...")
            break
        elif command == '\x03':  # Ctrl+C
            print("\nForced exit.")
            break
        else:
            print("\rInvalid command", end="", flush=True)

if __name__ == "__main__":
    main()