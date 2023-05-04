#!/usr/bin/env python3

import numpy as np

import truck_trailer_amr.helper_funcs_ctrl as helpers

DEADZONE = 0.05


class PositionStateFeedback:
    '''Perform feedback on the trailer position and orientation in two stages:
    1) compute a desired change in longitudinal and angular velocity of the
    trailer
    2) compute a desired change in longitudinal and angular velocity of the
    truck, based on the reference provided by 1).

    x1 = [px1, py1, theta1] --> to error in trajectory frame.

    du1 = [dv1, domega1] = - K * [dpx1, dpy1, dtheta1]
    where K is of the structure K = [Kpx, 0, 0;\
                                     0  Kpy, Ktheta]

    --> similar to Control Theory Exam assignment 5 with the swivel cart.

    du1 is provided as a reference for the truck, which is then projected
    onto the input space of the truck.

    Notation comments:

    - 'w' denotes "map" frame, 'tt' denotes 'trailer trajectory' frame (\
    i.e. the reference trajectory for the trailer) 't' denotes 'trailer'\
    frame, '0' denotes 'truck' (= vehicle 0) frame
    - 'L' means perpendicular component
    - 'll' means parallel component
    - Vectors and angles are expressed in the trajectory frame, unless\
    specified otherwise.\
    e.g. ex0_v == ex0_tt.
    - a trailing '_v' denotes 2D vectors in cartesian space.

    '''
    def __init__(self, params):
        '''Constructor for PositionStateFeedback class.
        '''
        self.Kt_fwd = np.array(params['Kfb_trailer_forward'])
        self.Kt_bckwd = np.array(params['Kfb_trailer_backward'])
        self.deadzone = params['fb_deadzone']
        self.Kw = np.array(params['Kfb_truck'])
        self.L1 = params['vehicles']['trailer1']['L']

    def get_fb_cmd(self, state_x, state_x_ref, u0ff):
        '''Get the error in the trailer traj frame (tt), perform two-step
        feedback and return the feedback input command.

        :param state_x: full state of shape [px1, py1, theta1, theta0, omega0]
        :type state_x: np.array

        :param state_x_ref: reference state of shape\
                           [px1, py1, theta1, theta0, omega0]
        :type state_x_ref: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :return: (du0, v1_fb_v, v0_ff_v, v0_fb_v)

            - du0 (np.array) - truck input correction of shape [dv0, domega0]
            - v1_fb_v (np.array) trailer fb velocity of shape [vx, vy]
            - v0_ff_v (np.array) truck ff velocity of shape [vx, vy]
            - v0_fb_v (np.array) truck fb velocity of shape [vx, vy]
        '''
        # Preperation: get rotation matrices and unit vectors.
        R_w_to_tt = helpers.get_rotation_matrix(theta=state_x_ref[2])
        dx_tt = self.get_trailer_err_traj_frame(state_x, state_x_ref,
                                                R_w_to_tt)
        R_tt_to_t = helpers.get_rotation_matrix(theta=dx_tt[2])
        R_tt_to_0 = helpers.get_rotation_matrix(theta=dx_tt[3])
        R_t_to_tt = R_tt_to_t.T
        R_0_to_tt = R_tt_to_0.T
        ex1_ = R_t_to_tt[:, 0]
        ex0_v = R_0_to_tt[:, 0]
        # import pdb; pdb.set_trace
        # Two-step feedback.
        du1 = self.get_trailer_input_cmd(dx_tt, u0ff, state_x_ref)

        du0, dv0_v = self.get_truck_input_cmd(u0ff, dx_tt, du1, ex0_v,
                                              R_t_to_tt)
        # Get the corresponding velocities.
        v1_fb_v, v0_ff_v, v0_fb_v = self.get_world_frame_velocity_components(
                                    R_w_to_tt, u0ff, dv0_v, du1, ex0_v, ex1_)

        return du0, v1_fb_v, v0_ff_v, v0_fb_v

    def get_trailer_err_traj_frame(self, state_x, state_x_ref, R_w_to_tt):
        '''From the current state and reference state, get
        the error expressed in the tt frame.

        :param state_x: state of shape [px1, py1, theta1, theta0, omega0]
        :type state_x: np.array

        :param state_x_ref: reference state of shape\
                           [px1, py1, theta1, theta0, omega0]
        :type state_x_ref: np.array

        :return: dx_tt - the error vector expressed in the tt frame,\
                         of shape [dpx1, dpy1, dtheta1, dtheta0, domega0]
        :rtype: np.array
        '''
        dx_tt = np.zeros(state_x.shape)
        dx_tt[0:2] = R_w_to_tt @ (state_x[0:2] - state_x_ref[0:2])
        dx_tt[2] = state_x[2] - state_x_ref[2]
        # Express all vectors in tt frame, therefore -state_x_ref[2]
        dx_tt[3] = state_x[3] - state_x_ref[2]
        # Remove 2*pi jumps due to wrapping or having driven multiple rounds
        # in the past.
        dx_tt[2] = (((dx_tt[2]) - np.pi) % (2*np.pi)) - np.pi
        dx_tt[3] = (((dx_tt[3]) - np.pi) % (2*np.pi)) - np.pi

        dx_tt[4] = state_x[4] - state_x_ref[4]

        return dx_tt

    def get_trailer_input_cmd(self, dx_tt, u0ff, _):
        '''Perform state feedback on the trailer position and orientation
        error to correct towards the trajectory.

        :param dx_tt: error expresed in tt frame, of shape\
                      [dpx1, dpy1, dth1, dth0]
        :type dx_tt: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :return: du1 - trailer correction input of shape [dv1x, dv1y]
        :rtype: np.array
        '''
        # Gain on y-error flips sign depending on the direction of travel
        Kt = self.Kt_fwd if np.sign(u0ff[0]) >= 0 else self.Kt_bckwd
        du1 = - Kt @ dx_tt[0:3]

        return du1

    def get_truck_input_cmd(self, u0ff, dx_tt, du1, ex0_v,
                            R_t_to_tt):
        '''Compute the input commands to the truck to control towards the
        reference of the trailer.

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param dx_tt: error expresed in tt frame, of shape\
                      [dpx1, dpy1, dth1, dth0]
        :type dx_tt: np.array

        :param du1: trailer correction input of shape [dv1x, dv1y]
        :type du1: np.array

        :param ex0_v: unit vector along truck longitudinal axis
        :type ex0_v: np.array

        :param R_t_to_tt: Rotation matrix from t frame to tt frame
        :type R_t_to_tt: np.ndarray

        :return: (du0, dv0_v)

            - du0 (np.array) - truck correction input
            - dv0_v (np.array) - truck feedback velocity vector
        '''
        dv0_v = self.get_truck_feedback_velocity(du1, R_t_to_tt)

        v0ff_ = u0ff[0] * ex0_v
        dv0_clip_v = self.clip_fb_velocity(dv0_v, v0ff_, u0ff[0], ex0_v)
        # dv0_clip_v = dv0_v

        du0 = self.get_truck_twist(dv0_clip_v, ex0_v, u0ff, dx_tt[4])

        return du0, dv0_v

    def get_truck_feedback_velocity(self, du1, R_t_to_tt):
        '''From the feedback velocity of the trailer and the rotation of the
        trailer in the trailer trajectory frame, get the desired feedback
        velocity of the truck.

        :param du1: trailer control input correction
        :type du1: np.array

        :param R_t_to_tt: Rotation matrix from t frame to tt frame
        :type R_t_to_tt: np.ndarray

        :return: dv0_v - truck feedback velocity vector
        :rtype: np.array
        '''
        # In the trailer frame
        dv0_ll_t = du1[0]
        dv0_L_t = du1[1] * self.L1

        # Transform to the trailer trajectory frame
        dv0_v = R_t_to_tt @ np.array([dv0_ll_t, dv0_L_t])

        return dv0_v

    def get_truck_twist(self, dv0_v, ex0_v, u0ff, domega0=0):
        '''From the feedback velocity vector, the current orientation of the
        truck and the feedforward control input, get the desired longitudinal
        velocity increment and the angular speed increment of the truck.

        :param dv0_v: truck feedback velocity vector
        :type dv0_v: np.array

        :param ex0_v: unit vector along truck longitudinal axis
        :type ex0_v: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param domega0: truck rotational velocity correction
        :type domega0: float

        :return: du0 - truck correction input
        :rtype: float
        '''
        # Longitudinal velocity - project onto the current direction
        dv0_proj = dv0_v @ ex0_v
        # Angle error feedback
        v0_ = u0ff[0] * ex0_v
        v0tot_ = v0_ + dv0_v
        dth0 = np.arctan2(np.cross(v0tot_, v0_), np.dot(v0tot_, v0_))
        dth0 = ((dth0 - np.pi) % (2*np.pi)) - np.pi
        # Make a deadzone to avoid exaggerated turning close to standstill.
        dw0 = - self.Kw * dth0 if abs(u0ff[0]) > DEADZONE else 0.
        # Put together in control input vector.
        du0 = np.array([dv0_proj, dw0])

        return du0

    def clip_fb_velocity(self, dv0_v, v0_ff_v, v0_ff, ex0_v):
        '''Only allow the feedback to have a limited effect on ff+fb.
        If the parallel fb component is larger and opposite in sign compared to
        the feedforward, then limit parallel feedback component in magnitude
        to that of the feedforward.

        :param dv0_v: truck feedback velocity vector
        :type dv0_v: np.array

        :param v0_ff_v: truck feedforward velocity vector
        :type v0_ff_v: np.array

        :param v0_ff: truck feedforward velocity vector magnitude
        :type v0_ff: float

        :param ex0_v: unit vector along truck longitudinal axis
        :type ex0_v: np.array

        :return: dv0_clip_v - clipped truck velocity correction
        :rtype: np.array
        '''
        tol = 1e-5

        # Get parallel component along truck longitudinal axis.
        dv0_ll = dv0_v @ ex0_v
        dv0_ll_ = dv0_ll * ex0_v
        # Get perpendicular component.
        dv0_L_ = dv0_v - dv0_ll_

        # Clip parallel component if the sign flips after adding fb to ff.
        if np.sign(v0_ff + dv0_ll) != np.sign(v0_ff):
            # Don't make ff + fb exactly 0 for numeric stability.
            dv0_ll_clip = - v0_ff + np.sign(v0_ff) * tol
        else:
            dv0_ll_clip = dv0_ll
        dv0_ll_clip_ = dv0_ll_clip * ex0_v
        # Clip perpendicular component
        dv0_L_mag = np.linalg.norm(dv0_L_)
        dv0_L_clip = min(dv0_L_mag, abs(dv0_ll_clip))
        if dv0_L_mag == 0:
            dv0_L_mag += 1e-5
        dv0_L_clip_ = dv0_L_clip * dv0_L_ / dv0_L_mag

        # Recombine parallel and perpendicular.
        dv0_clip_v = dv0_ll_clip_ + dv0_L_

        return dv0_clip_v

    def get_world_frame_velocity_components(self, R_w_to_tt, u0ff, dv0_v, du1,
                                            ex0_v, ex1_):
        '''From the x-axes of trailer and truck expressed in the trailer
        trajectory (tt) frame, get the velocity vectors of
        - feedforward of truck
        - feedback of truck
        - feedback of trailer

        :param R_w_to_tt: rotation matrix from world to tt frame
        :type R_w_to_tt: np.ndarray

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param dv0_v: truck feedback velocity vector of shape [dv0x, dv0y]
        :type dv0_v: np.array

        :param du1: trailer feedback control input
        :type du1: np.array

        :param ex0_v, ex1_: unit x vectors of respectively truck and trailer,
                        expressed in tt frame, of shape [x, y]
        :type ex0_v, ex1_: np.array

        :return: (v1_fb_v, v0_ff_v, v0_fb_v)

            - v1_fb_v (np.array) - trailer feedback velocity vector
            - v0_ff_v (np.array) - truck feedforward velocity vector
            - v0_fb_v (np.array) - truck feedback velocity vector
        '''
        v0_ff_v = R_w_to_tt.T @ (u0ff[0] * ex0_v)
        v0_fb_v = R_w_to_tt.T @ dv0_v
        v1_fb_v = R_w_to_tt.T @ (du1[0] * ex1_)

        return v1_fb_v, v0_ff_v, v0_fb_v


class PosSFVarGains(PositionStateFeedback):
    '''A small upgrade from the PositionStateFeedback:
    The same two-step feedback, but the trailer feedback gains are a function
    of the feedforward velocity, to take into account the sensitivity of the
    vehicle dynamics to varying velocity.
    '''
    def __init__(self, params):
        '''Constructor for PosSFVarGains class.

        :param params: controller parameters
        :type params: dict
        '''
        self.Ktcoeffspos = np.array(params['Kfb_coeffs_pos'])
        self.Ktcoeffsneg = np.array(params['Kfb_coeffs_neg'])

        self.Kw = np.array(params['Kfb_truck'])

        self.L1 = params['vehicles']['trailer1']['L']
        self.M0 = params['vehicles']['trailer1']['M']

    def get_trailer_input_cmd(self, dx_tt, u0ff, state_x_ref):
        '''Perform state feedback on the trailer position and orientation
        error to correct towards the trajectory.

        :param dx_tt: error expresed in tt frame, of shape\
                      [dpx1, dpy1, dth1, dth0]
        :type dx_tt: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param state_x_ref: reference state of shape\
                           [px1, py1, theta1, theta0, omega0]
        :type state_x_ref: np.array

        :return: du1 = feedback control input for trailer of shape [dv1x, dv1y]
        :rtype: np.array
        '''
        v1ff = self.get_trailer_ff_vel(u0ff, state_x_ref)
        Kt = self.get_trailer_gain(v1ff)
        du1 = - Kt @ dx_tt[0:3]

        return du1

    def get_trailer_gain(self, v0ff):
        '''Get the gain for the trailer feedback controller as function of the
        feedforward velocity.

        :param v0ff: truck feedforward velocity magnitude
        :type v0ff: float

        :return: K - feedback gain matrix
        :rtype: np.ndarray
        '''
        Kcoeffs = self.Ktcoeffspos if v0ff >= 0 else self.Ktcoeffsneg
        K = np.zeros((2, 3))
        Ni = K.shape[0]
        Nj = K.shape[1]
        for i in range(Ni):
            for j in range(Nj):
                K[i, j] = Kcoeffs[i, j, :] @ np.array([v0ff, 1])
        return K

    def get_trailer_ff_vel(self, u0, state_x_ref):
        '''Use the kinematic relation to get trailer feedforward velocity from
        the current state and feedforward input.

        NOT IMPLEMENTED - just take truck feedforward velocity as parameter.

        :param u0: truck feedforward input command
        :type u0: np.array

        :return: v1 - trailer feedforward velocity
        '''
        # beta01 = state_x_ref[3] - state_x_ref[2]
        # print('th0 - th1', state_x_ref[3], state_x_ref[3])
        # v1 = u0[0]*np.cos(beta01) + self.M0*np.sin(beta01)*u0[1]
        # print('v0, v1', u0[0], v1)

        # Workaround for not having theta0 reference: take v0 as the scheduling
        # parameter instead of v1. This is a close approximation for small
        # beta01, e.g. driving on a straight line or circle with large
        # curvature.
        v1 = u0[0]

        return v1


class PosSFDamped(PositionStateFeedback):
    '''Position state feedback control with variable gains, with additional
    damping on the truck orientation control, to cope with the finite time of
    the Kelo wheel to reach its desired orientation.
    '''
    def __init__(self, params):
        '''Constructor for the PosSFVarGainsDamped class.

        :param params: controller parameters
        :type params: dict
        '''
        self.Kt_fwd = np.array(params['Kfb_trailer_forward'])
        self.Kt_bckwd = np.array(params['Kfb_trailer_backward'])

        self.Kw = np.array(params['Kfb_truck'])

        self.L1 = params['vehicles']['trailer1']['L']
        self.M0 = params['vehicles']['trailer1']['M']

    def get_truck_twist(self, dv0_v, ex0_v, u0ff, domega0=0):
        '''From the feedback velocity vector, the current orientation of the
        truck and the feedforward control input, get the desired longitudinal
        velocity increment and the angular speed increment of the truck.

        :param dv0_v: truck feedback velocity vector
        :type dv0_v: np.array

        :param ex0_v: unit vector along truck longitudinal axis
        :type ex0_v: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param domega0: truck rotational velocity correction
        :type domega0: float

        :return: du0 - truck correction input
        :rtype: float

        '''
        # Longitudinal velocity - project onto the current direction
        dv0_proj = dv0_v @ ex0_v
        # Angle error feedback
        v0_ = u0ff[0] * ex0_v
        v0tot_ = v0_ + dv0_v
        dth0 = np.arctan2(np.cross(v0tot_, v0_), np.dot(v0tot_, v0_))
        dth0 = ((dth0 - np.pi) % (2*np.pi)) - np.pi

        dx0 = np.array([dth0, domega0])

        # Make a deadzone to avoid exaggerated turning close to standstill.
        dw0 = - self.Kw @ dx0 if abs(u0ff[0]) > DEADZONE else 0.
        # Put together in control input vector.
        du0 = np.array([dv0_proj, dw0])

        return du0


class PosSFVarGainsDamped(PosSFVarGains):
    '''Position state feedback control with variable gains, with additional
    damping on the truck orientation control, to cope with the finite time of
    the Kelo wheel to reach its desired orientation.
    '''
    def __init__(self, params):
        '''Constructor for the PosSFVarGainsDamped class.
        '''
        self.Ktcoeffspos = np.array(params['Kfb_coeffs_pos'])
        self.Ktcoeffsneg = np.array(params['Kfb_coeffs_neg'])

        self.Kw = np.array(params['Kfb_truck'])

        self.L1 = params['vehicles']['trailer1']['L']
        self.M0 = params['vehicles']['trailer1']['M']

    def get_truck_twist(self, dv0_v, ex0_v, u0ff, domega0=0):
        '''From the feedback velocity vector, the current orientation of the
        truck and the feedforward control input, get the desired longitudinal
        velocity increment and the angular speed increment of the truck.

        :param dv0_v: truck feedback velocity vector
        :type dv0_v: np.array

        :param ex0_v: unit vector along truck longitudinal axis
        :type ex0_v: np.array

        :param u0ff: truck feedforward control input of shape [v0, omega0]
        :type u0ff: np.array

        :param domega0: truck rotational velocity correction
        :type domega0: float

        :return: du0 - truck correction input
        :rtype: float

        '''
        # Longitudinal velocity - project onto the current direction
        dv0_proj = dv0_v @ ex0_v
        # Angle error feedback
        v0_ = u0ff[0] * ex0_v
        v0tot_ = v0_ + dv0_v
        dth0 = np.arctan2(np.cross(v0tot_, v0_), np.dot(v0tot_, v0_))
        dth0 = ((dth0 - np.pi) % (2*np.pi)) - np.pi

        dx0 = np.array([dth0, domega0])

        # Make a deadzone to avoid exaggerated turning close to standstill.
        dw0 = - self.Kw @ dx0 if abs(u0ff[0]) > DEADZONE else 0.
        # Put together in control input vector.
        du0 = np.array([dv0_proj, dw0])

        return du0
