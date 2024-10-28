import rclpy
import numpy as np
from nav_msgs.msg import Odometry


def odom_callback(msg: Odometry):

    """
    Process an Odometry message and extract the robot's state.

    This function takes an Odometry message and extracts the robot's position,
    orientation, linear velocity, and angular velocity into a single list.

    Args:
        msg (Odometry): The Odometry message to process.

    Returns:
        list: A list containing the robot's state in the following order:
            [0-2]:   Position (x, y, z)
            [3-6]:   Orientation (x, y, z, w) as quaternion
            [7-9]:   Linear velocity (x, y, z)
            [10-12]: Angular velocity (x, y, z)

    Note:
        - The orientation is represented as a quaternion (x, y, z, w).
        - All values are in the frame specified by the Odometry message.
        - Units are typically meters for positions and radians for orientations,
          but this can depend on the system's configuration.
    """

    pose = msg.pose.pose
    twist = msg.twist.twist
    state = [
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z
    ]

    return state
        

def quaternion_to_euler(q):

    """
    Converts quaternions to Euler angles (roll, pitch, yaw).

    Args:
    q (numpy.array): Array of quaternions.

    Returns:
    numpy.array: Array of Euler angles (roll, pitch, yaw).
    """

    # Extract the values from Q
    q0 = q[:, 3]
    q1 = q[:, 0]
    q2 = q[:, 1]
    q3 = q[:, 2]
        
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
        
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
    # 3x3 rotation matrix to Euler angles
    roll = np.arctan2(r21, r22)
    pitch = np.arcsin(-r20)
    yaw = np.arctan2(r10, r00)
        
    return np.column_stack((roll, pitch, yaw))


