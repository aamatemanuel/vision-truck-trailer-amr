#!/usr/bin/env python3
from bagpy import bagreader
import pandas as pd
import numpy as np
import json
from truck_trailer_amr.helper_funcs_ctrl import euler_from_quaternion

'''
This file reads .bag files and extracts desired data from specified topics,
defining a TopicHandler class. Formats data in pandas DataFrames

Available topics from rosbag:

/angle_encoder/angle_meas
/cmd_vel
/controller/truck_fb_vel
/controller/truck_fb_vel_vector
/controller/truck_ff_vel
/controller/truck_ff_vel_vector
/controller/all_corridor_contours
/controller/corridor_contours
/controller/corridor_targets
/controller/pose_est_amr
/controller/pose_est_trailer
/controller/setpoint
/controller/state_finish
/controller/trailer_fb_vel
/controller/trailer_fb_vel_vector
/controller/trailer_ff_vel
/controller/trigger_point
/controller/vehicle_x0
/eagle_ros/pose_est
/fsm/state
/fsm/task
/fsm/trigger_mp_drive
/map
/map_metadata
/motionplanner/calc_time
/motionplanner/desired_path
/motionplanner/real_path
/motionplanner/result
/motionplanner/stage1_path
/motionplanner/stage2_path
/motionplanner/stage3_path
/motionplanner/stage_time
/motionplanner/trigger
/rosout
/rosout_agg
/vive_localization/pose_est
'''


def get_data(datafile):
    '''
    Load the requested data and return it formatted in a dict.

    Args:
        - data_directory: str (ending with /)
        - datafile: str
        - extension: str (either .bag=slow or .json=fast)

    Returns:
        - data: dict
    '''
    data = {}

    # Just for printing the available topics
    dummy = TopicHandler(datafile)

    # ----------------------------
    data_setpoint = TopicHandler(
        datafile,
        '/controller/setpoint')
    data['t_setpoint'] = \
        data_setpoint.get_data_by_name('Time')
    data['setpoint_x'] = \
        data_setpoint.get_data_by_name('point.x')
    data['setpoint_y'] = \
        data_setpoint.get_data_by_name('point.y')
    # ----------------------------
    data_pose_est_trailer = TopicHandler(
        datafile,
        '/controller/pose_est_trailer')
    data['t_p1'] = \
        data_pose_est_trailer.get_data_by_name('Time')
    data['p1_x'] = \
        data_pose_est_trailer.get_data_by_name('pose.position.x')
    data['p1_y'] = \
        data_pose_est_trailer.get_data_by_name('pose.position.y')
    [quatx, quaty, quatz, quatw] = [
        data_pose_est_trailer.get_data_by_name('pose.orientation.x'),
        data_pose_est_trailer.get_data_by_name('pose.orientation.y'),
        data_pose_est_trailer.get_data_by_name('pose.orientation.z'),
        data_pose_est_trailer.get_data_by_name('pose.orientation.w')]
    quat = [[quatx[i], quaty[i], quatz[i], quatw[i]]
            for i in range(len(quatx))]
    data['theta1'] = [euler_from_quaternion(*quat[i])[2]  # get yaw
                      for i in range(len(quat))]
    # ----------------------------
    data_pose_est_amr = TopicHandler(
        datafile,
        '/controller/pose_est_amr')
    data['t_p0'] = \
        data_pose_est_amr.get_data_by_name('Time')
    data['p1_0'] = \
        data_pose_est_amr.get_data_by_name('point.x')
    data['p1_0'] = \
        data_pose_est_amr.get_data_by_name('point.y')
    [quatx, quaty, quatz, quatw] = [
        data_pose_est_amr.get_data_by_name('pose.orientation.x'),
        data_pose_est_amr.get_data_by_name('pose.orientation.y'),
        data_pose_est_amr.get_data_by_name('pose.orientation.z'),
        data_pose_est_amr.get_data_by_name('pose.orientation.w')]
    quat = [[quatx[i], quaty[i], quatz[i], quatw[i]]
            for i in range(len(quatx))]
    data['theta0'] = [euler_from_quaternion(*quat[i])[2]  # get yaw
                      for i in range(len(quat))]
    # ----------------------------
    data_comp_times = TopicHandler(
        datafile,
        '/motionplanner/calc_time')
    data['t_comp_times'] = \
        data_comp_times.get_data_by_name('Time')
    data['comp_times'] = \
        data_comp_times.get_data_by_name('data')
    # ----------------------------
    data_stage_time = TopicHandler(
        datafile,
        '/motionplanner/stage_time')
    data['t_stage_time'] = \
        data_stage_time.get_data_by_name('Time')
    data['stage1_time'] = \
        data_stage_time.get_data_by_name('T1')
    data['stage2_time'] = \
        data_stage_time.get_data_by_name('T2')
    data['stage3_time'] = \
        data_stage_time.get_data_by_name('T3')
    # ----------------------------
    data_corridor_change = TopicHandler(
        datafile,
        '/controller/corridor_change')
    data['t_corridor_change'] = \
        data_corridor_change.get_data_by_name('Time')
    data['corridor_change'] = \
        data_corridor_change.get_data_by_name('data')

    return data


def get_data_json(datafile):
    '''Load the json file that was created from the .bag-file with the same
    name.
    '''
    with open(datafile, "r") as openfile:
        data_json = json.load(openfile)

    return data_json


def get_data_selection(key, data, data_select, tstart, tend, only_first=False):
    '''For the given key, select the data recorded between tstart and tend.
    '''
    result = find_data_in_trange(
        data['t_' + key],
        data[key],
        tstart, tend, only_first=only_first)
    data_select['t_' + key] = np.append(
        data_select['t_' + key], result[0])
    data_select[key] = np.append(
        data_select[key], result[1])

    return data_select


def find_data_in_trange(t_array, array, tstart, tend, only_first=False):
    '''Given tstart and tend in trange=[tstart, tend],
    find the data in the array that falls within this range.
    To return only the first occurence (e.g. for collision detection that
    keeps on setting off), set only_first=True.

    Args:
        t_array: []
        array: []
        tstart: float
        tend: float
        only_first: bool

    returns:
        t_array_select: []
        array_select: []
    '''
    try:
        i_0 = next(k for k, val in enumerate(t_array) if val >= tstart)
        i_f = len(t_array) - next(k for k, val in enumerate(reversed(t_array))
                                  if val <= tend)

        if only_first:
            t_array_select = [t_array[i_0:i_f][0]]
            array_select = [array[i_0:i_f][0]]
        else:
            t_array_select = t_array[i_0:i_f]
            array_select = array[i_0:i_f]

    except Exception as e:
        t_array_select = []
        array_select = []

    return t_array_select, array_select


class TopicHandler:
    '''Keeps you shielded from the hassle of reading .bag files.

    NOTE: This does not work if the message type corresponding to the topic
    that you are tring to read has a list in it, e.g. (dummy example):

    PositionTrajectory.msg:

    float32[] px
    float32[] py
    float32[] pz

    TODO: figure out how to extract the data with lists.

    '''
    def __init__(self, filename, topic=None):
        '''
        '''
        b = bagreader(filename)
        if not topic:
            self.topic = None
            self.data = None
            self.keys = None

            print('Data file contains recordings from following topics:')
            for topic in b.topics:
                print(topic)
            return
        else:

            data = b.message_by_topic(topic)
            self.topic = topic
            try:
                self.data = pd.read_csv(data)
                self.keys = self.data.keys().to_list()
            except Exception:
                self.keys = []
                self.data = pd.DataFrame({})

    def get_keys(self):
        '''
        '''
        print('Keys for the topic ' + str(self.topic))
        print(self.keys)
        return self.keys

    def get_data_by_name(self, name):
        '''From the pandas.DataFrame, get the data vector by name.

        Args:
            data: pandas.DataFrame
            name: str

        Returns:
            numpy array
        '''
        try:
            i = self.keys.index(name)
            vec = self.data.values[:, i].tolist()
        except Exception:
            vec = []

        return vec
