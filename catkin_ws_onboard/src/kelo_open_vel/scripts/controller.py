#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node("controller")
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sample_time = 0.02
    ctrl_rate = rospy.Rate(1./sample_time)

    # CREATE INPUT SIGNAL
    tlen = 5.
    step = 2 * np.pi / 5
    vref = [0.2 for _ in range(int(tlen / sample_time))]
    omegaref = [0 for _ in range(int(tlen / sample_time))]

    input_cmd = np.zeros(2)
    i = 0

    while not (i >= len(vref) or rospy.is_shutdown()):
        print(i, vref[i], omegaref[i])
        input_cmd[0] = vref[i]
        input_cmd[1] = omegaref[i]
        print(input_cmd)
        
        # PUBLISH INPUT_CMD
        cmd_twist = Twist()
        cmd_twist.linear.x = input_cmd[0]
        cmd_twist.angular.z = input_cmd[1]
        cmd_vel_pub.publish(cmd_twist)

        i += 1
        ctrl_rate.sleep()

    for i in range(int(2/sample_time)):
        cmd_vel_pub.publish(Twist())
        ctrl_rate.sleep()

