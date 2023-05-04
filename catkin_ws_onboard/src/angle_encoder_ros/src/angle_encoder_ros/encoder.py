#!/usr/bin/env python3

from std_msgs.msg import Float32, Empty

import odroid_wiringpi as wpi
import threading
import numpy as np
import rospy

TICKS_PER_REV = 2048
CL = 31
DA = 30
PUB_RATE = 50.


class Encoder:
    '''A class that reads out angle encoder measurements and broadcasts them
    on a ROS topic.
    '''
    def __init__(self):
        '''Constructor for EncoderBridge.
        '''
        rospy.init_node('angle_encoder')

        self.pub_rate = rospy.Rate(PUB_RATE)
        self.counter = 0
        self.angle_pub = rospy.Publisher('/angle_encoder/angle_meas',
                                         Float32, queue_size=1)
        rospy.Subscriber('/angle_encoder/reset_encoder',
                         Empty, self.reset_encoder)

        wpi.wiringPiSetup()
        wpi.pinMode(DA, 0)
        wpi.pinMode(CL, 0)
        self.clkLastState = wpi.digitalRead(CL)

        self.counter_thread = threading.Thread(target=self.count)

    def start(self):
        '''Start measurement loop.
        '''
        self.counter_thread.start()
        rospy.loginfo('ENC  - Angle encoder node active')

        while not rospy.is_shutdown():
            # Convert to radians and publish
            angle_rad = self.get_angle_from_counter()
            self.angle_pub.publish(angle_rad)
            self.pub_rate.sleep()

    def count(self):
        '''Continuously count angle encoder ticks.
        '''
        while not rospy.is_shutdown():
            clkState = wpi.digitalRead(CL)
            daState = wpi.digitalRead(DA)
            if clkState != self.clkLastState:
                if daState != clkState:
                    self.counter -= 1
                else:
                    self.counter += 1
            self.clkLastState = clkState

    def get_angle_from_counter(self):
        '''Convert the encoder counter value to an angle in radians.

        :return: angle in radians
        :rtype: float

        '''
        angle_rad = 2*np.pi / TICKS_PER_REV * self.counter

        return angle_rad

    def reset_encoder(self, _=None):
        '''Set the encoder counter value to 0 to define the zero rad point.

        :param _: empty message
        :type _: std_msgs.msg.Empty
        '''
        rospy.loginfo('ENC  - Angle encoder reset')
        self.counter = 0


if __name__ == '__main__':
    encoder_bridge = Encoder()
    encoder_bridge.start()
