#!/usr/bin/env python3
from std_msgs.msg import Float32, Empty

from truck_trailer_amr.speed_estimator import SpeedEstimator

import rospy
from collections import deque
from scipy.signal import lfilter


class EncoderBridge:
    '''Class to receive angle encoder measurements from angle_encoder_ros
    and store them for use in controller. Also estimate encoder speed via
    finite difference on the measured angle, filtered with butterworth filter.
    '''
    def __init__(self, params):
        '''Constructor for EncoderBridge class.
        '''
        self.monitor_rate = rospy.Rate(5.)
        self.angle_meas = None

        ctrl_params = rospy.get_param('/controller')
        enc_params = rospy.get_param('/angle_encoder')

        params = {
            "sample_time": enc_params["sample_time"],
            "butter_order": ctrl_params["encoder_speed"]["butter_order"],
            "butter_cutoff": ctrl_params["encoder_speed"]["butter_cutoff"]
            }

        self.enc_speed_estimator = SpeedEstimator(params)
        self.enc_speed_est = 0.

        self.encoder_reset_pub = rospy.Publisher(
            '/angle_encoder/reset_encoder', Empty, queue_size=1)

        rospy.Subscriber('/angle_encoder/angle_meas', Float32,
                         self.store_angle_meas)

    def wait_for_stream(self):
        '''Block other operations until measurement has been received.
        '''
        rospy.loginfo('ENCB - Waiting for measurements')
        while self.angle_meas is None:
            self.monitor_rate.sleep()
        rospy.loginfo('ENCB - Encoder bridge ready')

    def get_angle_meas(self):
        '''Return latest pose estimate.
        '''
        return self.angle_meas

    def store_angle_meas(self, angle_meas):
        '''Callback for /angler_encoder/angle_meas topic to store latest
        encoder measurement. Also estimate the angular speed.

        :param angle_meas: angle measurement
        :type angle_meas: std_msgs.msg.Float32
        '''
        self.angle_meas = angle_meas.data
        self.enc_speed_est = self.enc_speed_estimator.estimate_vel(
                                                            self.angle_meas)[0]

    def reset_encoder(self):
        '''Publish message for angle_encoder to reset the counter as
        calibration.
        '''
        self.encoder_reset_pub.publish(Empty())
