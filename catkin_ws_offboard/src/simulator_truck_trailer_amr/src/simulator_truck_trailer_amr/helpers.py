from geometry_msgs.msg import Pose2D
import numpy as np


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    sr2 = np.sin(roll/2)
    cr2 = np.cos(roll/2)
    sp2 = np.sin(pitch/2)
    cp2 = np.cos(pitch/2)
    sy2 = np.sin(yaw/2)
    cy2 = np.cos(yaw/2)

    qx = sr2 * cp2 * cy2 - cr2 * sp2 * sy2
    qy = cr2 * sp2 * cy2 + sr2 * cp2 * sy2
    qz = cr2 * cp2 * sy2 - sr2 * sp2 * cy2
    qw = cr2 * cp2 * cy2 + sr2 * sp2 * sy2

    return [qx, qy, qz, qw]
