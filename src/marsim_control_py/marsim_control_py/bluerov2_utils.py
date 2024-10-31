from std_msgs.msg import Float64
import numpy as np


def publish_thruster_commands(thruster_pubs, u):

    """
    Publish thrust commands to the BlueROV2's six thrusters.

    This function takes a list of thruster publishers and a list of thrust commands,
    clips the thrust values to a safe range, and publishes them to the respective thrusters.

    Args:
        thruster_pubs (list): A list of ROS2 publishers for each thruster.
            Each publisher should be of type rclpy.publisher.Publisher[std_msgs.msg.Float64].
        u (list): A list of thrust commands for each thruster.
            The length of this list should match the number of publishers in thruster_pubs.

    Returns:
        None

    Note:
        - Thrust commands are clipped to the range [-5.0, 5.0] before publishing.
        - The function assumes that the order of thruster_pubs matches the order of thrust commands in u.
        - It's the caller's responsibility to ensure that the lengths of thruster_pubs and u match.
        - Thrust units are typically in Newtons or percentage of maximum thrust, depending on the ROV's configuration.

    Raises:
        IndexError: If the length of u is greater than the length of thruster_pubs.

    Example:
        thruster_publishers = [pub1, pub2, pub3, pub4, pub5, pub6]
        thrust_commands = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]
        publish_thruster_commands(thruster_publishers, thrust_commands)
    """

    for i, thrust in enumerate(u):
        if thrust is not None:
            msg = Float64()
            msg.data = float(np.clip(thrust, -5.0, 5.0))  # Clip thrust to -5 to +5 range
            thruster_pubs[i].publish(msg)