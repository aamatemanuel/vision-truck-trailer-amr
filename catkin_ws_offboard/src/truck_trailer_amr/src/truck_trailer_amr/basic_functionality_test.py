#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy

rospy.init_node('truck_trailer_amr')
rospy.loginfo('Truck-Trailer AMR node initialized')

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

ctrl_rate = rospy.Rate(10)

cmd_vel = Twist()
cmd_vel.linear.x = 0.1
# cmd_vel.linear.y = 1
# cmd_vel.linear.z = 1  # does nothing
# cmd_vel.angular.x = 1  # does nothing
# cmd_vel.angular.y = 1  # does nothing
cmd_vel.angular.z = 0.5

rospy.loginfo('Start publishing in 1s')
rospy.sleep(1.)
rospy.loginfo('Go!')
for i in range(10*10):
    if rospy.is_shutdown():
        break
    cmd_vel_pub.publish(cmd_vel)
    print(i)
    ctrl_rate.sleep()
rospy.loginfo('Done!')
cmd_vel_pub.publish(Twist())
