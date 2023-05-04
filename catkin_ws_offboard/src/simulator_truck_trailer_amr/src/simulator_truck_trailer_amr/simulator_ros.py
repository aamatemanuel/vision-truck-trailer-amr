#!/usr/bin/env python3
from geometry_msgs.msg import Pose2D, PoseStamped, Twist, Quaternion
from std_msgs.msg import Float32, Empty, Bool

import rospy
import numpy as np
import simulator_truck_trailer_amr.helpers as helpers
from simulator_truck_trailer_amr. simulator import \
    simulator_omega_init, simulate


class SimulatorROS:
    '''ROS wrapper for basic dynamics simulator.
    '''
    def __init__(self):
        '''Constructor for SimulatorROS.
        '''
        rospy.init_node('simulator')
        params = rospy.get_param('/simulator')
        self.update_rate = rospy.Rate(params['update_rate'])
        self.dt = 1/params['update_rate']
        # N trailers --> 0=truck, 1=trailer1, 2=trailer2, ...
        # x = [pxN, pyN, thetaN, thetaN-1, ..., theta1, theta0]
        self.x = np.array([params['init_cond']['px1'],
                           params['init_cond']['py1'],
                           params['init_cond']['theta1'],
                           params['init_cond']['theta0']])
        self.u = np.zeros(2)

        self.discrete_system_dynamics = simulator_omega_init(params)

        self.eagle_pub = rospy.Publisher(
            '/eagle_ros/pose_est', Pose2D, queue_size=1)
        self.vive_pub = rospy.Publisher(
            '/vive_localization/pose_est', PoseStamped, queue_size=1)
        self.eagle_pub3d = rospy.Publisher(
            '/eagle_ros/pose_est3d', PoseStamped, queue_size=1)
        self.angle_enc_pub = rospy.Publisher(
            '/angle_encoder/angle_meas', Float32, queue_size=1)
        self.healthy_pub = rospy.Publisher(
            '/vive_localization/healthy', Bool, queue_size=0)

        rospy.Subscriber('/cmd_vel', Twist, self.store_control_input)
        rospy.Subscriber('/simulator/reset', Empty, self.reset_simulator)

    def start(self):
        '''Run the simulator at the defined update rate.
        '''
        rospy.loginfo('SIM  - Simulator running')
        while not rospy.is_shutdown():
            # Publish new pose measurement
            self.publish_localization_pose()
            # Publish new angle encoder measurement
            theta0 = self.x[-1]
            theta1 = self.x[-2]
            beta01 = theta0 - theta1
            self.angle_enc_pub.publish(beta01)

            # Fake unhealthy measurements
            # self.healthy_pub.publish(False)

            # Simulate dynamics
            self.x = simulate(self.discrete_system_dynamics,
                              self.x, self.u, self.dt).full()

            self.update_rate.sleep()

    def store_control_input(self, input_cmd):
        '''Store an incoming /cmd_vel from controller to use in simulation.

        :param input_cmd: Twist message from controller
        :type input_cmd: geometry_msgs.msg.Twist
        '''
        self.u = np.array([input_cmd.linear.x, input_cmd.angular.z])

    def publish_localization_pose(self):
        '''Publish pose for controller measurement and visualization in Rviz.
        '''
        pose3D = PoseStamped()
        pose3D.header.stamp = rospy.Time.now()
        pose3D.header.frame_id = "map"
        pose3D.pose.position.x = self.x[0]
        pose3D.pose.position.y = self.x[1]
        pose3D.pose.orientation = Quaternion(*helpers.quaternion_from_euler(
                                                            0, 0, self.x[2]))

        self.eagle_pub.publish(Pose2D(*self.x[0:3]))
        self.vive_pub.publish(pose3D)
        # self.eagle_pub3d.publish(pose3D)

    def reset_simulator(self, _):
        '''Reset simulator to initial conditions.

        :param _: Empty message
        :type _: std_msgs.msg.Empty
        '''
        params = rospy.get_param('/simulator')
        self.x = np.array([params['init_cond']['px1'],
                           params['init_cond']['py1'],
                           params['init_cond']['theta1'],
                           params['init_cond']['theta0']])
        self.u = np.zeros(2)
        rospy.loginfo('SIM  - Reset simulator')


if __name__ == '__main__':
    simulator = SimulatorROS()
    simulator.start()
