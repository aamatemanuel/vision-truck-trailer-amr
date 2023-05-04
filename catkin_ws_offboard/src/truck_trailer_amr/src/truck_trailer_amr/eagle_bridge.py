#!/usr/bin/env python3
from geometry_msgs.msg import Pose2D

import rospy


class EagleBridge:
    '''Class to receive pose measurements from eagle_ros and store them for
    use in controller.
    '''
    def __init__(self):
        '''Constructor for EagleBridge class.
        '''
        self.healthy = True
        self.monitor_rate = rospy.Rate(5.)
        self.pose_est = Pose2D()

        rospy.Subscriber('/eagle_ros/pose_est', Pose2D, self.store_pose_est)

    def wait_for_stream(self):
        '''Block other operations until a measurement has been received.
        '''
        rospy.loginfo('EAGB - Waiting for measurements')
        while self.pose_est == Pose2D():
            self.monitor_rate.sleep()
        rospy.loginfo('EAGB - Eagle bridge ready')

    def store_pose_est(self, pose_est):
        '''Callback for /eagle_ros/pose_est topic to store latest pose
        estimate.

        :param pose_est: Pose estimate coming from eagle_ros node.
        :type pose_est: geometry_msgs.msg.PoseStamped
        '''
        self.pose_est = pose_est
