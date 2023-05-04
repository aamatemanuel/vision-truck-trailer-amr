#!/usr/bin/env python3
from collections import deque
from scipy.signal import butter, tf2ss
import numpy as np


class SpeedEstimator:
    '''Speed estimation with Nth order butterworth filter with coefficients
    A, B on the finite difference of the measured angle.
    '''
    def __init__(self, params):
        '''Constructor for the SpeedEstimator.

        :param params: parameters
        :type params: dict
        '''
        self.Ts = params["sample_time"]
        N = params['butter_order']
        wn = params['butter_cutoff']

        b, a = butter(N, wn, fs=1/self.Ts)
        self.A, self.B, self.C, self.D = tf2ss(b, a)

        self.X = np.zeros(self.A.shape[1])
        self.prev_meas = 0.

        self.vel_est = 0.

    def estimate_vel(self, meas):
        '''Propagate the finite difference of the measurements
        through the butterworth filter.

        :param meas: 1d measurement
        :type meas: float

        :return: vel_est - estimate of the velocity
        :rtype: np.array
        '''
        FD = np.array([meas - self.prev_meas]) / self.Ts
        self.X = self.A @ self.X + self.B @ FD
        vel_est = self.C @ self.X + self.D @ FD

        self.prev_meas = meas

        return vel_est
