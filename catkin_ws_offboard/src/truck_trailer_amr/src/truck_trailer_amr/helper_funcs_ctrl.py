#!/usr/bin/env python3
from geometry_msgs.msg import Point, Pose2D, Vector3, TransformStamped
from truck_trailer_amr.msg import Trigger_mp, TruckTrailerAmrState

import rospy
import numpy as np
import math


##############
# Saturation #
##############
def clip_inputs(input_cmd, ctrl_upper_limit, ctrl_lower_limit):
    '''Restricts the input signals to the maximal allowed value.

    :param input_cmd: Control input command
    :type input_cmd: np.array

    :param ctrl_upper_limit: Upper limit for the control input command
    :type ctrl_upper_limit: np.array

    :param ctrl_lower_limit: Lower limit for the control input command
    :type ctrl_lower_limit: np.array

    :return: input_cmd = the clipped control input command,
             saturation = whether or not one or more control inputs saturated
    :rtype: np.array, bool
    '''
    saturation = False
    input_cmd = np.minimum(np.maximum(input_cmd,
                           ctrl_lower_limit),
                           ctrl_upper_limit)

    if (input_cmd == ctrl_upper_limit).any() or (
            input_cmd == ctrl_lower_limit).any():
        saturation = True

    return input_cmd, saturation


#########################
# Trajectory inspection #
#########################

def reset_mp_traj():
    '''Return an empty mp_traj.

    :return: mp_traj, an empty trajectory
    :rtype: dict
    '''
    mp_traj = {
        'px1_coarse': [], 'py1_coarse': [],
        'px1': [], 'py1': [],
        'theta1_coarse': [], 'theta0_coarse': [],
        'theta1': [], 'theta0': [],
        'v0': [], 'omega0': [],
        'v0_coarse': [], 'omega0_coarse': [],
        'stage_indices': [],
        't': [], 't_coarse': []}

    return mp_traj


def select_input_command(index, mp_traj):
    '''Selects the desired input command from the calculated trajectories.
    returns input_cmd = np.array([p, q, r, T])

    :param index: Index along the trajectory from which to sample
    :type index: int

    :param mp_traj: trajectory from which to sample
    :type mp_traj: dict

    :return: (cmd, state_x_ref)

            - cmd (np.array) - control input command of shape [p, q, r, at],
            - state_x_ref (np.array) - state reference of shape\
                [px1, py1, theta1, theta0, omega0]
    '''
    cmd = np.array([mp_traj['v0'][index],
                    mp_traj['omega0'][index]])

    state_x_ref = np.array([mp_traj['px1'][index],
                            mp_traj['py1'][index],
                            mp_traj['theta1'][index],
                            mp_traj['theta0'][index],
                            mp_traj['omega0'][index]])

    return cmd, state_x_ref


def store_new_mp_traj(new_data):
    '''If mp_traj is empty, store the new data in it.

    :param new_data: the new solution coming in from motionplanner.
    :type new_data: truck_trailer_amr.msg.Trajectories

    :return: new_traj - the trajectories from new_data, formatted in the\
             way controller uses it.
    :rtype: dict
    '''
    new_traj = {
        't_coarse':
            list(new_data.t_coarse),
        't':
            list(new_data.t),
        'px1_coarse':
            list(new_data.px1_coarse),
        'py1_coarse':
            list(new_data.py1_coarse),
        'px1':
            list(new_data.px1),
        'py1':
            list(new_data.py1),
        'theta1_coarse':
            list(new_data.theta1_coarse),
        'theta0_coarse':
            list(new_data.theta0_coarse),
        'theta1':
            list(new_data.theta1),
        'theta0':
            list(new_data.theta0),
        'v0':
            list(new_data.v0),
        'omega0':
            list(new_data.omega0),
        'v0_coarse':
            list(new_data.v0_coarse),
        'omega0_coarse':
            list(new_data.omega0_coarse),
        'stage_indices':
            [state_index for state_index in new_data.stage_indices]}
    return new_traj


def combine_old_new_mp_traj(new_data, old_traj, indices):
    '''Add new trajectory to old one starting from index and removing
    past part of trajectory.

    :param new_data: the new solution coming in from motionplanner.
    :type new_data: truck_trailer_amr.msg.Trajectories

    :param old_traj: the previous stored trajectory
    :type old_traj: dict

    :param indices: the indices from which to start the combined solution and\
                    where to stitch the new solution to the old one
    :type indices: int[]

    :return: new_traj - the combined old and new trajectories
    :rtype: dict
    '''
    prev_index, index = indices

    new_traj = {
        't_coarse':
            old_traj['t'][prev_index:index] +
            [val + old_traj['t'][index] for val in new_data.t_coarse],
        't':
            old_traj['t'][prev_index:index] +
            [val + old_traj['t'][index] for val in new_data.t],
        'px1_coarse':
            old_traj['px1'][prev_index:index] +
            list(new_data.px1_coarse),
        'py1_coarse':
            old_traj['py1'][prev_index:index] +
            list(new_data.py1_coarse),
        'px1':
            old_traj['px1'][prev_index:index] +
            list(new_data.px1),
        'py1':
            old_traj['py1'][prev_index:index] +
            list(new_data.py1),
        'theta1_coarse':
            old_traj['theta1'][prev_index:index] +
            list(new_data.theta1_coarse),
        'theta0_coarse':
            old_traj['theta0'][prev_index:index] +
            list(new_data.theta0_coarse),
        'theta1':
            old_traj['theta1'][prev_index:index] +
            list(new_data.theta1),
        'theta0':
            old_traj['theta0'][prev_index:index] +
            list(new_data.theta0),
        'v0':
            old_traj['v0'][prev_index:index] +
            list(new_data.v0),
        'omega0':
            old_traj['omega0'][prev_index:index] +
            list(new_data.omega0),
        'v0_coarse':
            old_traj['v0'][prev_index:index] +
            list(new_data.v0_coarse),
        'omega0_coarse':
            old_traj['omega0'][prev_index:index] +
            list(new_data.omega0_coarse),
        'stage_indices':
            [(index-prev_index)+state_index for state_index in
             new_data.stage_indices]}

    return new_traj


def trim_last_sol(traj):
    '''Remove last xxx elements of the trajectory, because they belong
    to a closed corridor, and don't make sense at all.

    .. todo::
      make 'xxx=1' a parameter.

    :param traj: the stored trajectory
    :type traj: dict

    :return traj: the trimmed trajectory
    :rtype: dict

    '''
    traj['px1'] = traj['px1'][0:-1]
    traj['py1'] = traj['py1'][0:-1]
    traj['theta1'] = traj['theta1'][0:-1]
    traj['theta0'] = traj['theta0'][0:-1]
    traj['v0'] = traj['v0'][0:-1]
    traj['omega0'] = traj['omega0'][0:-1]
    traj['t'] = traj['t'][0:-1]

    return traj


def get_trigger_at_index(mp_traj, index):
    '''Get the trigger point given the full trajectory and the index.

    :param mp_traj: the stored trajectory
    :type mp_traj: dict

    :param index: the index at which to get the trigger
    :type index: int

    :return: (trigger, index)

        - trigger (truck_trailer_amr.msg.Trigger_mp) - trigger for mp
        - index (int) - (corrected) index at which trigger was sampled from\
                        the trajectory
    '''
    T_fine_desired = mp_traj['t'][index]

    return (Trigger_mp(u_t0=[mp_traj['v0'][index],
                             mp_traj['omega0'][index]],
                       x_t0=[mp_traj['px1'][index],
                             mp_traj['py1'][index],
                             mp_traj['theta1'][index],
                             mp_traj['theta0'][index]]),
            index)

    # # find index of closest time in coarse time
    # index_coarse = next(i for i, T_coarse in enumerate(mp_traj['t_coarse'])
    #                     if T_coarse >= T_fine_desired)
    # # print('t_coarse_full',mp_traj['t_coarse'])
    # T_coarse = mp_traj['t_coarse'][index_coarse]
    # # print('T_coarse met index',T_coarse, index_coarse)
    # index_fine_corrected = np.argmin(abs(np.array(mp_traj['t']) - T_coarse))
    # # print('index_fine_corrected',index_fine_corrected)
    #
    # # index_coarse = np.array(mp_traj['t_coarse']) - t_fine
    #
    # return (Trigger_mp(u_t0=[mp_traj['v0_coarse'][index_coarse],
    #                         mp_traj['omega0_coarse'][index_coarse]],
    #                   x_t0=[mp_traj['px1_coarse'][index_coarse],
    #                         mp_traj['py1_coarse'][index_coarse],
    #                         mp_traj['theta1_coarse'][index_coarse],
    #                         mp_traj['theta0_coarse'][index_coarse]]),
    #         index_fine_corrected)


def get_traj_pose_at_index(index, traj):
    '''Get the trailer pose on mp_traj at given index.

    :param index: index at which to get the pose from the trajectory
    :type: int

    :param traj: stored trajectory
    :type traj: dict

    :return: pose at index - [px1, py1, theta1]
    :rtype: float[]
    '''
    return [traj['px1'][index], traj['py1'][index], traj['theta1'][index]]


def select_position_setpoint(index, mp_traj):
    '''Select the current position setpoint on the tracked trajectory.

    :param index: index at which to get the setpoint from the trajectory
    :type: int

    :param traj: stored trajectory
    :type traj: dict

    :return: pos_setpoin - position setpoint of the shape [px1, py1, pz1]
    :rtype: geometry_msgs.msg.Point

    '''
    pos_setpoint = Point(mp_traj['px'][index],
                         mp_traj['py'][index],
                         mp_traj['pz'][index])
    return pos_setpoint


def find_closest_point(pose, traj):
    '''Return the index of the point on the given reference path that is
    closest in space (2-norm) to the given point.

    :param pose: the current pose, compared to the trajectory
    :type pose: geometry_msgs.msg.Pose2D

    :param traj: the trajectory on which to find the closest point
    :type traj: dict

    :return: index_closest - the index corresponding to the closest point on\
             the trajectory
    :rtype: int
    '''
    x = np.array(traj['px1']) - pose.position.x
    y = np.array(traj['py1']) - pose.position.y
    index_closest = np.argmin(x*x + y*y)

    return index_closest


def get_mean(point1, point2):
    '''Get the mean of two points.

    :param point1: first point
    :type point1: geometry_msgs.msg.Point

    :param point2: second point
    :type point2: geometry_msgs.msg.Point

    :return: x, y, z - the mean of the two points
    :rtype: float, float, float
    '''
    x = (point1.x + point2.x)/2
    y = (point1.y + point2.y)/2
    z = (point1.z + point2.z)/2
    return x, y, z


##################
# msg operations #
##################

def sum_vectors(vector1, vector2):
    '''Sum two vectors.

    :param vector1: first vector
    :type vector1: geometry_msgs.msg.Vector3

    :param vector2: second vector
    :type vector2: geometry_msgs.msg.Vector3

    :return: vectorsum - the vector sum of the given vectors
    :rtype: geometry_msgs.msg.Vector3
    '''
    vectorsum = Vector3(x=vector1.x + vector2.x,
                        y=vector1.y + vector2.y,
                        z=vector1.z + vector2.z)

    return vectorsum


def translate_point(point, vector):
    '''Translate the given point along the given vector.

    :param point: point to translate
    :type point: geometry_msgs.msg.Point

    :param vector: vector to translate by
    :type vector: geometry_msgs.msg.Vector3

    :return: translated point
    :rtype: geometry_msgs.msg.Point
    '''
    translated_point = Point(x=point.x + vector.x,
                             y=point.y + vector.y,
                             z=point.z + vector.z)

    return translated_point


def copy_pose(pose):
    '''Copy the point without reference (such that later alterations don't
    influence the original point).

    :param pose: pose to copy
    :type pose: geometry_msgs.msg.Pose2D

    :return: new_pose
    :rtype: geometry_msgs.msg.Pose2D
    '''
    new_pose = Pose2D(x=pose.x,
                      y=pose.y,
                      theta=pose.theta)
    return new_pose


def copy_state(state):
    '''Copy the point without reference (such that later alterations don't
    influence the original point).

    :param state: state to be copied
    :type state: truck_trailer_amr.msg.TruckTrailerAmrState()
    :return: new_state - copied state
    :rtype: truck_trailer_amr.msg.TruckTrailerAmrState()
    '''
    new_state = TruckTrailerAmrState(px1=state.px1,
                                     py1=state.py1,
                                     theta1=state.theta1,
                                     theta0=state.theta0)
    return new_state


def position_diff_norm(point1, point2):
    '''Returns the 2 norm of the difference vector between two given points.

    :param point1: first point
    :type point1: geometry_msgs.msg.Point

    :param point2: second point
    :type point2: geometry_msgs.msg.Point

    :return: norm - norm of difference
    :rtype: float

    '''
    norm = np.linalg.norm(np.array([point1.x, point1.y])
                          - np.array([point2.x, point2.y]))
    return norm


def pose_to_tf(pose, child_frame_id):
    '''Get the transform corresponding to the pose.

    :param pose: given pose
    :type pose: geometry_msgs.msg.PoseStamped

    :param child_frame_id: name of the child frame
    :type child_frame_id: str

    :return: transf - the corresponding transform
    :rtype: geometry_msgs.msg.TransformStamped
    '''
    transf = TransformStamped()
    transf.header.stamp = rospy.Time.now()
    transf.header.frame_id = pose.header.frame_id
    transf.child_frame_id = child_frame_id
    transf.transform.translation = pose.pose.position
    transf.transform.rotation = pose.pose.orientation

    return transf


###################
# Transformations #
###################
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    :param roll: The roll (rotation around x-axis) angle in radians.
    :type roll: float

    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :type pitch: float

    :param yaw: The yaw (rotation around z-axis) angle in radians.
    :type yaw: float

    :return: (qx, qy, qz, qw)

        - The orientation in quaternion [x,y,z,w] format

    :rtype: float[]
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


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)

    :param x, y, z, w: Quaternion
    :type x, y, z, w: float

    :return: (roll_x, pitch_y, yaw_z)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def get_truck_pose(trailer_pose, beta01, veh_params):
    '''Given the pose of the trailer and the angle between

    :param trailer_pose: trailer pose
    :type trailer_pose: geometry_msgs.msg.Pose2D

    :param beta01: angle between truck and trailer
    :type beta01: float

    :param M1: distance from rear axle to hitching point
    :type M1: float

    :param L1: length of trailer
    :type L1: float

    :return: truck_pose - pose of the truck, based on trailer state and angle
    :rtype: geometry_msgs.msg.Pose2D
    '''
    L1 = veh_params['trailer1']['L']
    M1 = veh_params['trailer1']['M']
    truck_pose = Pose2D(x=trailer_pose.x + L1 * np.cos(trailer_pose.theta)
                                         + M1 * np.cos(trailer_pose.theta),
                        y=trailer_pose.y + L1 * np.sin(trailer_pose.theta)
                                         + M1 * np.sin(trailer_pose.theta),
                        theta=trailer_pose.theta + beta01)
    return truck_pose


def get_rotation_matrix(theta):
    '''Given the angle, return the rotation matrix.

    :param theta: angle to calculate rotation matrix from
    :type theta: float

    :return: R - rotation matrix
    :rtype: np.array
    '''
    R = np.array([[np.cos(theta), np.sin(theta)],
                  [-np.sin(theta), np.cos(theta)]])

    return R
