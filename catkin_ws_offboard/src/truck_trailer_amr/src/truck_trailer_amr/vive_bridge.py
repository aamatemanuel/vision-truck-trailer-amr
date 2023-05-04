#!/usr/bin/env python3
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Bool

import truck_trailer_amr.helper_funcs_ctrl as helpers
from truck_trailer_amr.speed_estimator import SpeedEstimator

import rospy
import numpy as np

MONITOR_RATE = 5.


class ViveBridge:
    '''Class to receive pose measurements from vive_localization and store them
    for use in controller.
    '''
    def __init__(self):
        '''Constructor for ViveBridge class.
        '''
        self.monitor_rate = rospy.Rate(MONITOR_RATE)
        self.pose_est = Pose2D(x=1e10)
        self.healthy = True

        ctrl_params = rospy.get_param('/controller')
        vive_params = rospy.get_param('/vive_localization')

        params = {
            "sample_time": vive_params["sample_time"],
            "butter_order": ctrl_params["trailer_speed"]["butter_order"],
            "butter_cutoff": ctrl_params["trailer_speed"]["butter_cutoff"]
            }

        self.yaw_vel_estimator = SpeedEstimator(params)
        self.yaw_vel_est = 0.
        self.prev_yaw_meas = 0.

        rospy.Subscriber(
            '/vive_localization/pose_est', PoseStamped, self.store_pose_est)
        rospy.Subscriber(
            '/vive_localization/healthy', Bool, self.vive_health_check)

    def wait_for_stream(self):
        '''Block other operations until a measurement has been received.
        '''
        rospy.loginfo('VIBR - Waiting for measurements')
        while self.pose_est == Pose2D(x=1e10) and not rospy.is_shutdown():
            self.monitor_rate.sleep()
        rospy.loginfo('VIBR - Vive bridge ready')

    def store_pose_est(self, pose_est):
        '''Callback for /vive_localization/pose_est topic to store latest pose
        estimate.

        :param pose_est: Pose estimate coming from vive_localization node.
        :type pose_est: geometry_msgs.msg.PoseStamped
        '''
        quat = [pose_est.pose.orientation.x,
                pose_est.pose.orientation.y,
                pose_est.pose.orientation.z,
                pose_est.pose.orientation.w]
        _, _, yaw = helpers.euler_from_quaternion(*quat)

        self.pose_est = Pose2D(x=pose_est.pose.position.x,
                               y=pose_est.pose.position.y,
                               theta=yaw)

        # Smoothly varying yaw measurement
        yaw = np.unwrap([self.prev_yaw_meas, yaw])[-1]
        self.prev_yaw_meas = yaw
        # Filter the FD
        self.yaw_vel_est = self.yaw_vel_estimator.estimate_vel(yaw)[0]

    def vive_health_check(self, healthy):
        '''Store the health information coming from the vive localization node.

        :param healthy: Health status of vive_localization
        :type healthy: std_msgs.msg.Bool
        '''
        self.healthy = healthy.data
