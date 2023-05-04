#!/usr/bin/env python3

from truck_trailer_amr.msg import (Trigger_mp, Trajectories,
                                   TruckTrailerAmrState)
from geometry_msgs.msg import (Twist, Pose2D, PointStamped, Point,
                               Vector3Stamped, Vector3)
from std_msgs.msg import (Bool, Empty, String, Header, Int32)
from std_srvs.srv import Trigger

import rospy
import scipy.io as io
import numpy as np

from truck_trailer_amr.rviz_handler import RvizHandler
from truck_trailer_amr.corridor_handler import CorridorHandler
from truck_trailer_amr.eagle_bridge import EagleBridge
from truck_trailer_amr.vive_bridge import ViveBridge
from truck_trailer_amr.encoder_bridge import EncoderBridge
from truck_trailer_amr.feedback_control import (
    PositionStateFeedback, PosSFVarGains, PosSFVarGainsDamped, PosSFDamped)
import truck_trailer_amr.helper_funcs_ctrl as helpers
import truck_trailer_amr.helper_funcs_mp as helpers_mp


class Controller:
    '''A class for continuous control, performing actions corresponding to the
    state dictated by FSM (fsm.py). This class is a ROS node that communicates
    with FSM, motionplanner, sensors and actuators.
    '''
    ############################
    # Initialization functions #
    ############################
    def __init__(self):
        '''Initialization of Controller object.
        '''
        rospy.init_node("controller")

        self.state = "initialization"
        self.state_dict = {
            "standby":
                {"action": self.safety_brake},
            "brake":
                {"action": self.safety_brake},
            "encoder calib maneuver":
                {"action": self.drive_fwd_stop},
            "ff traj drive":
                {"action": self.drive_ff_traj},
            "mp standby":
                {"action": self.mp_wait},
            "p2p":
                {"action": self.p2p_drive},
            "warehouse drive multistage":
                {"action": self.warehouse_drive_multistage},
            "identify velocity":
                {"action": self.identify_velocity},
            "test feedback control":
                {"action": self.test_fb_ctrl},
            "test wheel slip":
                {"action": self.test_wheel_slip},
             }

        self.navigation_states = {"mp standby", "p2p",
                                  "warehouse drive multistage"}

        ctrl_params, mp_params = self._init_params()

        self.rviz_handler = RvizHandler()
        self.corridor_handler = CorridorHandler()
        self.encoder_bridge = EncoderBridge(ctrl_params)

        # self.pos_fb = PositionStateFeedback({**ctrl_params, **mp_params})
        # self.pos_fb = PosSFDamped({**ctrl_params, **mp_params})
        # self.pos_fb = PosSFVarGains({**ctrl_params, **mp_params})
        self.pos_fb = PosSFVarGainsDamped({**ctrl_params, **mp_params})

        if ctrl_params["localization_system"] == "Eagles":
            self.loc_bridge = EagleBridge()
        elif ctrl_params["localization_system"] == "HTC Vive":
            self.loc_bridge = ViveBridge()
        else:
            rospy.logerr("CTRL - Specify either 'Eagles' or 'HTC Vive' as " +
                         "localization_system in controller_config.yaml")
            raise NameError("Invalid localization type")

        self._init_variables()
        self._init_msg_srv()

    def _init_params(self, _=None):
        '''Initializes externally configurable parameters
        (rosparams).

        :param _: placeholder for empty topic callback, defaults to None
        :type _: std_msgs.msg.Empty, optional

        :return: (ctrl_params, mp_params)

            - ctrl_params (dict) - Controller parameters from rosparam server
            - mp_params (dict) - Motionplanner parameters from rosparam server
        '''

        ctrl_params = rospy.get_param('/controller')
        mp_params = rospy.get_param('/motionplanner')

        try:
            self.record_directory = ctrl_params['record_directory']
            self.plot_stages = ctrl_params['plot_stages']
            self.sample_time = ctrl_params['sample_time']
            self.ctrl_rate = rospy.Rate(1./self.sample_time)

            self.relative_p2p_goal = ctrl_params['relative_p2p_goal']
            self.mp_update_point = ctrl_params['mp_update_point']
            self.mp_update_index = ctrl_params['mp_update_index']
            self.mp_comp_timeout = self.mp_update_index * self.sample_time
            self.ctrl_upper_limit = np.array([ctrl_params['max_v_wheel_input'],
                                              ctrl_params['max_omega_input']])
            self.ctrl_lower_limit = np.array([
                                             -ctrl_params['max_v_wheel_input'],
                                             -ctrl_params['max_omega_input']])
            self.jackknife_threshold = ctrl_params['jackknife_threshold']

            self.service_timeout = ctrl_params['service_timeout']
            self.pos_nrm_tol = ctrl_params['goal_reached_pos_tol']
            self.orient_tol = ctrl_params['goal_reached_orient_tol']

            self.veh_params = mp_params['vehicles']
            self.opt_params = mp_params['opti']

            rospy.loginfo("CTRL - Parameters loaded")

        except Exception as e:
            rospy.logerr("CTRL - Parameter(s) missing, " +
                         "check the config yaml file.\n %s" % e)

        return ctrl_params, mp_params

    def _init_variables(self, _=None):
        '''Initializes variables.
        '''

        # FSM State related variables
        self.mp_target_reached = False
        self.at_final_destination = False
        self.setpoint_reached = False
        self.state_changed = False
        self.executing_state = False
        self.state_killed = False
        self.overtime = False
        self.mp_drive_monitor_raised = False

        # Other
        self.mp_calc_succeeded = True
        self.mp_fire_time = rospy.Time.now()
        self.mp_init = True
        self.mp_index = 0
        self.mp_prev_sol_index = 0
        self.mp_monitor_index = 0
        self.corridor_checkpoint_index = 0
        self.mp_goal = np.array([None, None, None, None])
        self.mp_trigger = Trigger_mp(x_t0=[0., 0., 0., 0.])
        self.mp_trigger_index = 0
        self.mp_traj = helpers.reset_mp_traj()
        self.platform_driver_status = False

    def _init_msg_srv(self):
        '''Initializes rostopic Publishers and Subscribers and service proxies.
        '''
        self.mp_trigger_pub = rospy.Publisher(
            '/motionplanner/trigger', Trigger_mp, queue_size=1)
        self.state_finish_pub = rospy.Publisher(
            '/controller/state_finish', Empty, queue_size=1)
        self.goal_reached_pub = rospy.Publisher(
            '/controller/goal_reached', Bool, queue_size=1)
        self.corridor_checkpoint_pub = rospy.Publisher(
            '/controller/checkpoint', PointStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher(
            '/controller/setpoint', PointStamped, queue_size=1)
        self.trigger_point_pub = rospy.Publisher(
            '/controller/trigger_point', PointStamped, queue_size=1)
        self.trigger_fsm_mp_drive = rospy.Publisher(
            '/fsm/trigger_mp_drive', Empty, queue_size=1)
        self.truck_fb_vel_pub = rospy.Publisher(
            '/controller/truck_fb_vel_vector', Vector3Stamped, queue_size=1)
        self.truck_ff_vel_pub = rospy.Publisher(
            '/controller/truck_ff_vel_vector', Vector3Stamped, queue_size=1)
        self.trailer_fb_vel_pub = rospy.Publisher(
            '/controller/trailer_fb_vel_vector', Vector3Stamped, queue_size=1)

        # Kelo interface
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/platform_driver/status', Int32,
                         self.get_kelo_status)

        rospy.Subscriber('/motionplanner/result', Trajectories,
                         self.get_mp_result)
        rospy.Subscriber('/fsm/state', String, self.switch_state)
        rospy.Subscriber('/fsm/update_rosparams', Empty, self.update_params)

    def _init_fsm_handshake_srv(self):
        '''Provide handshake service such that FSM knows controller is running.
        '''
        self.fsm_handshake_srv = rospy.Service(
            "/controller/fsm_handshake", Trigger, self.fsm_handshake)

    def update_params(self, _=None):
        '''Callback to update parameters at runtime.

        :param _: Placeholder for empty topic callback, defaults to None
        :type _: std_msgs.msg.Empty
        '''
        self._init_params()

    #################
    # Main function #
    #################
    def start(self):
        '''Configures, starts the controller's main loop.
        '''
        self.loc_bridge.wait_for_stream()
        self.encoder_bridge.wait_for_stream()
        self.wait_for_kelo_status()

        self._init_fsm_handshake_srv()
        rospy.Subscriber(
            '/motionplanner/goal', TruckTrailerAmrState, self.set_mp_goal)
        rospy.Subscriber(
            '/motionplanner/relative_goal', TruckTrailerAmrState,
            self.set_mp_relative_goal)

        self.rviz_handler.draw_room_contours()
        self.rviz_handler.reset_traj_markers()
        rospy.loginfo('CTRL - Controller running')

        # Run main loop.
        self.main()

    def main(self):
        '''Main control loop, executing FSM-state actions.
        '''
        while not rospy.is_shutdown():
            if self.state_changed:
                self.state_changed = False
                rospy.loginfo('CTRL - State changed to: ' + str(self.state))

                # ======================================
                # Execute state action (function).
                self.executing_state = True
                self.state_dict[self.state]["action"]()
                self.executing_state = False
                # ======================================

                # State has not finished if it has been killed!
                if not self.state_killed:
                    self.state_finish_pub.publish(Empty())
                    rospy.loginfo('CTRL - State finished')
                self.state_killed = False

                if self.state == "standby":
                    # Do some actions when going into standby state.
                    self.corridor_handler.reset()

            # Actions that are repeatedly executed in standby state.
            self.visualize_poses()
            self.ctrl_rate.sleep()

    ##############
    # Navigation #
    ##############
    def warehouse_drive_multistage(self):
        '''Drive along a route, automatically switching between setpoints.
        '''
        self.mp_traj = helpers.reset_mp_traj()
        self.mp_index = 0
        self.mp_init = True
        self.mp_monitor_index = 1

        # First corridor
        wp = self.corridor_handler.get_corridor_target()
        self.set_mp_goal(TruckTrailerAmrState(*wp))
        self.fire_motionplanner()
        self.mp_drive()

        # Go over all remaining corridors.
        while not (rospy.is_shutdown() or self.state_killed):

            wp = self.corridor_handler.get_corridor_target()
            self.mp_goal = wp

            if not self.corridor_handler.last():
                self.mp_drive()

            if (not self.mp_target_reached) or self.corridor_handler.last():
                break

        self.corridor_handler.load_next_route()
        self.overtime = False

    def p2p_drive(self):
        '''Drive only from start to end point with motionplanner.
        '''
        if None in self.mp_goal:
            rospy.logwarn("CTRL - No mp goal set yet. Will not start p2p.")
            return

        # Initialize
        self.mp_traj = helpers.reset_mp_traj()
        self.mp_index = 0
        self.mp_prev_sol_index = 0
        self.mp_monitor_index = 1
        self.mp_init = True

        # First motionplanner call
        self.fire_motionplanner()
        self.mp_drive()

        self.overtime = False

    def drive_ff_traj(self):
        '''Get stored trajectory and apply the inputs in feedforward.
        '''
        # traj = rospy.get_param('/traj')

        import yaml
        import rospkg
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        pck_path = rospack.get_path('truck_trailer_amr')

        traj = open(pck_path + "/data/truck_trailer_x_u_forward.yaml")
        traj = yaml.load(traj)

        v_wheel_ref_traj = traj['u']['v_l']
        omega_ref_traj = traj['u']['omega']
        rospy.loginfo('CTRL - Start feedforward trajectory')
        rospy.sleep(1.)
        for i in range(len(v_wheel_ref_traj)):
            input_cmd = np.array([v_wheel_ref_traj[i], omega_ref_traj[i]])
            self.send_input_cmd(input_cmd)
            self.ctrl_rate.sleep()
        rospy.loginfo('CTRL - At end of trajectory')

    ########################
    # Navigation functions #
    ########################
    def set_mp_goal(self, goal, relative=False, trigger=None):
        '''Sets the goal and fires motionplanner.

        :param goal: target end state
        :type goal: truck_trailer_amr.msg.TruckTrailerAmrState

        :param relative: relative or absolute goal, defaults to False
        :type relative: bool, optional

        :param trigger: mp trigger info at update point, defaults to None
        :type trigger: Trigger_mp, optional
        '''

        if not (self.state in self.navigation_states):
            rospy.logwarn('CTRL - Activate a navigation task before ' +
                          'setting motion planner goal!')
            return

        rospy.loginfo('CTRL - New mp goal set')

        self.mp_drive_monitor_raised = False
        self.at_final_destination = False
        self.mp_init = True
        self.rviz_handler.reset_traj_markers()

        self.mp_goal = np.array([goal.px1, goal.py1,
                                 goal.theta1, goal.theta0])
        if relative:
            trailer_pose = self.loc_bridge.pose_est
            self.mp_goal[0] += trailer_pose.x
            self.mp_goal[1] += trailer_pose.y
            self.mp_goal[2] += trailer_pose.theta
            self.mp_goal[3] = (self.mp_goal[2] +
                               self.encoder_bridge.angle_meas)

        # Trigger fsm to go to p2p_drive state.
        self.trigger_fsm_mp_drive.publish(Empty())

    def set_mp_relative_goal(self, goal):
        '''Sets the goal and fires motionplanner.

        :param goal: relative goal
        :type goal: truck_trailer_amr.msg.TruckTrailerAmrState
        '''
        self.set_mp_goal(goal=goal, relative=True)

    def mp_wait(self):
        '''Wait in place until a solution arrives.
        '''
        self.state_finish_pub.publish(Empty())
        self.safety_brake()

        while not (rospy.is_shutdown() or self.state_killed):
            self.visualize_poses()
            self.ctrl_rate.sleep()

    def mp_drive(self):
        '''Perform mp_ctrl_updates to fly from current position to mp_goal.
        Alternately monitor status and update control.
        '''
        if self.mp_drive_monitor():
            return

        while not (rospy.is_shutdown() or self.state_killed):
            # Do mp_ctrl_update first to put input command at start of the
            # control interval.
            self.mp_ctrl_update()
            if self.mp_drive_monitor():
                return
            self.ctrl_rate.sleep()

    def mp_ctrl_update(self):
        '''Control updates in navigation state.

          - Updates the controller with newly calculated trajectories and
            velocity commands.
          - Sends out new velocity command.

        '''
        # Wait for first result. While calculating, stay in place.
        if self.mp_init:
            return

        # Retrieve the new feedforward input command.
        (cmd_ff, state_x_ref) = helpers.select_input_command(
                                                   self.mp_index, self.mp_traj)
        # Do control update (ff & fb).
        self.ctrl_update(cmd_ff, state_x_ref)

        # Visualize in rviz.
        self.mp_visualize()
        setpoint = PointStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.point.x = state_x_ref[0]
        setpoint.point.y = state_x_ref[1]
        self.setpoint_pub.publish(setpoint)

        self.mp_index += 1

    ##############
    # Monitoring #
    ##############

    def mp_drive_monitor(self):
        '''During p2p driving, monitor
            - arrival
            - computation time (overtime)
            - at update point before new solution
            - sensor health
        '''
        pos_tol = self.pos_nrm_tol
        orient_tol = self.orient_tol

        if not len(self.mp_traj['px1']):
            update_point_state = self.get_state_x()
        else:

            update_index = min(self.mp_index+self.mp_update_index,
                               len(self.mp_traj['px1'])-1)
            update_point_state = np.array(
                                        [self.mp_traj['px1'][update_index],
                                         self.mp_traj['py1'][update_index],
                                         self.mp_traj['theta1'][update_index],
                                         self.mp_traj['theta0'][update_index]])

        end_of_traj = (self.mp_index >= len(self.mp_traj['px1']) and
                       self.mp_index > 0)

        # Monitor computation time.
        comp_time = (rospy.Time.now() - self.mp_fire_time).to_sec()
        self.overtime = (comp_time > self.mp_comp_timeout and
                         not self.at_final_destination)
        if self.overtime:
            rospy.logerr("CTRL - OVERTIME: Fire motionplanner timed out")

        # Monitor whether passed update point on trajectory (directly related
        # to overtime!).
        invalid_checkpoint = (self.mp_index >= self.mp_monitor_index and
                              not end_of_traj and
                              not self.at_final_destination)
        if invalid_checkpoint:
            rospy.logerr(
                    "CTRL - At update point before new traj")

        if not self.at_final_destination:
            self.at_final_destination = self.check_goal_reached(
                            self.corridor_handler.mp_last_goal,
                            update_point_state, self.pos_nrm_tol, orient_tol)
            if self.at_final_destination:
                rospy.loginfo('CTRL - Reached final destination.')
                self.mp_traj = helpers.trim_last_sol(self.mp_traj)
        else:
            self.rviz_handler.reset_velocity_markers()

        # Monitor localization system health
        if not self.loc_bridge.healthy:
            rospy.logwarn("CTRL - Localization not healthy!")

        # Monitor jackknifing
        if abs(self.encoder_bridge.angle_meas) > self.jackknife_threshold:
            rospy.logwarn("CTRL - Jackknifing!")
            jackknifing = True
        else:
            jackknifing = False

        # Safety precaution.
        going_to_crash = (self.overtime or
                          invalid_checkpoint or
                          not self.mp_calc_succeeded or
                          not self.loc_bridge.healthy or
                          jackknifing or
                          end_of_traj)

        if going_to_crash:
            self.dont()

        self.mp_drive_monitor_raised = going_to_crash

        return self.mp_drive_monitor_raised

    def get_mp_result(self, data):
        '''Store results of motionplanner calculations.

        :param data: calculated trajectories received from 'mp_result' topic,
                     published by motionplanner.
        :type data: truck_trailer_amr.msg.Trajectories
        '''
        # MP Index when receiving the solution. (copy to make sure it doesn't
        # change while processing the solution)
        mp_index = self.mp_index

        if (self.state not in self.navigation_states or
                self.at_final_destination or rospy.is_shutdown()):
            return

        # Check how long the mp computation took.
        self.mp_receive_time = rospy.Time.now()
        comp_time = (self.mp_receive_time - self.mp_fire_time).to_sec()
        self.mp_calc_succeeded = data.success
        rospy.loginfo("CTRL - New traj (" + str(round(comp_time, 3)) + " s)")

        # Check feasibility of received trajectory.
        if not self.mp_calc_succeeded:
            rospy.logwarn("CTRL - Motion planner unsuccessful")
            # Keep on trying to find a solution.
            # self.fire_motionplanner()
            # Monitor will interrupt.
            return

        # Store the new trajectory.
        if self.state in self.navigation_states and self.state != "mp standby":
            # Slice and paste together
            if not self.mp_traj['t']:
                self.mp_traj = helpers.store_new_mp_traj(data)
            else:
                self.mp_traj = helpers.combine_old_new_mp_traj(
                            data, self.mp_traj, [0, self.mp_trigger_index])

            # TODO(self.mp_traj['stage_indices'][0])
            self.corridor_checkpoint_index = 0

            # Set monitor index
            self.mp_monitor_index = min(mp_index + self.mp_update_index,
                                        len(self.mp_traj['px1'])-1)
        else:
            self.mp_traj = {
                't': data.t, 't_coarse': data.t_coarse,
                'px1_coarse': data.px1_coarse, 'py1_coarse': data.py1_coarse,
                'px1': data.px1, 'py1': data.py1, 'theta1': data.theta1,
                'theta0': data.theta0, 'theta1_coarse': data.theta1_coarse,
                'theta0_coarse': data.theta0_coarse, 'v0': data.v0,
                'omega0': data.omega0, 'v0_coarse': data.v0_coarse,
                'omega0_coarse': data.omega0_coarse}

        # First iteration? --> stay, start from the beginning of the traj.
        # If not: Take finite computation time between firing and receiving
        # into account.
        if self.mp_init:
            self.mp_index = 0
            mp_index = 0
            self.mp_init = False

            if (mp_index + self.mp_update_index >=
                    self.corridor_checkpoint_index):
                self.corridor_checkpoint_index = 0
                # self.corridor_handler.toggle()
                self.mp_goal = self.corridor_handler.get_corridor_target()

            self.mp_trigger_index = min(self.mp_update_index,
                                        len(self.mp_traj['px1'])-1)
            self.mp_trigger, self.mp_trigger_index = \
                helpers.get_trigger_at_index(
                    self.mp_traj, self.mp_trigger_index)

        elif self.state != "mp standby":
            # ? mp_index + self.mp_update_index
            index_start = self.mp_trigger_index

            # Toggle check + perform toggle if possible.
            self.mp_goal = self.corridor_handler.toggle_check(
                self.mp_traj, index_start, mp_index,
                self.mp_update_index, self.mp_goal)

            # Get new trigger, mp_update_index into the future.
            # Prevent selecting an update index beyond mp_traj horizon.
            self.mp_trigger_index = min(
                mp_index + self.mp_update_index, len(self.mp_traj['px1'])-1)

            self.mp_trigger, self.mp_trigger_index = \
                helpers.get_trigger_at_index(self.mp_traj,
                                             self.mp_trigger_index)

        # Re-fire motionplanner.
        # ===========================
        # ENABLE MPC UPDATES
        # Do not trigger motionplanner if monitor was raised or state was
        # changed.
        if self.mp_drive_monitor_raised or \
                self.state not in self.navigation_states:
            return
        else:
            self.fire_motionplanner(self.mp_trigger)
        # ===========================

        # Visualize.
        if self.state == "warehouse drive multistage":
            N_1 = self.opt_params['N_1']
            N_2 = self.opt_params['N_2']
            N_3 = self.opt_params['N_3']
            self.rviz_handler.publish_desired_multistage(
                data.px1_coarse[0:N_1+1],
                data.py1_coarse[0:N_1+1],
                data.px1_coarse[N_1+1:N_1+N_2+2],
                data.py1_coarse[N_1+1:N_1+N_2+2],
                data.px1_coarse[N_1+N_2+2:],
                data.py1_coarse[N_1+N_2+2:])
        self.rviz_handler.publish_desired(self.mp_traj['px1_coarse'][:],
                                          self.mp_traj['py1_coarse'][:],
                                          0)
        self.rviz_handler.publish_desired_fine(
            self.mp_traj['px1'][:],
            self.mp_traj['py1'][:],
            0)

    def fire_motionplanner(self, trigger=None):
        '''Publishes inputs to motionplanner via Trigger topic.
        If trigger is not provided, then fire with current estimated state.

        :param trigger: trigger to send to motionplanner, defaults to None
        :type trigger: truck_trailer_amr.msg.Trigger_mp, optional
        '''
        rospy.loginfo('CTRL - Fire motionplanner')

        self.mp_fire_time = rospy.Time.now()
        if trigger:
            trigger.x_tf = self.mp_goal.tolist()
            trigger.u_tf = [0., 0.]
        else:
            trigger = Trigger_mp(u_t0=[0., 0.],
                                 u_tf=[0., 0.],
                                 x_t0=self.get_state_x().tolist()[:-1],
                                 x_tf=self.mp_goal.tolist())

        if self.state == "warehouse drive multistage":
            (trigger.w0_stage1, trigger.w0_stage2, trigger.w0_stage3,
             trigger.w1_stage1, trigger.w1_stage2, trigger.w1_stage3) = \
                self.corridor_handler.get_active_corridors_vectors()
            trigger.x_tm = \
                self.corridor_handler.get_active_corridors_targets()[0]
            trigger.direction = \
                self.corridor_handler.get_active_corridors_directions()

        if not self.at_final_destination:
            self.mp_trigger_pub.publish(trigger)

    ###########
    # Control #
    ###########
    def ctrl_update(self, cmd_ff, state_x_ref):
        '''Control updates to combine feedforward command with feedback based
        on state estimates.

        - check localization health
        - get state
        - get control input command
        - apply control input

        :param cmd_ff: feedforward control input command of shape [v0, w0]
        :type cmd_ff: np.array

        :param state_x_ref: state reference to track of shape
                            [px1, py1, theta1, theta0, omega0]
        :type state_x_ref: np.array
        '''
        # Check the health of the localization system. If not healthy, do not
        # perform the control action.
        if not self.loc_bridge.healthy:
            rospy.logerr("CTRL - Localization not healthy")
            return

        state_x = self.get_state_x()
        # Get feedback command
        cmd_fb, v1_fb, v0_ff, v0_fb = self.pos_fb.get_fb_cmd(
                                                  state_x, state_x_ref, cmd_ff)

        # Combine feedforward + feedback
        input_cmd = cmd_ff + cmd_fb
        # input_cmd = cmd_ff

        # Limit the inputs to the maximal specified value.
        input_cmd, saturation = helpers.clip_inputs(input_cmd,
                                                    self.ctrl_upper_limit,
                                                    self.ctrl_lower_limit)
        if saturation:
            rospy.logwarn('CTRL - Control input saturation')

        # Send control input sample.
        self.send_input_cmd(input_cmd)

        self.visualize_velocities(v1_fb, v0_ff, v0_fb)
        self.visualize_poses()

    #################
    # Kelo commands #
    #################
    def safety_brake(self):
        '''Brake as emergency measure: give 0 velocity reference.
        '''
        self.cmd_vel_pub.publish(Twist())

    def send_input_cmd(self, input_cmd):
        '''Set AMR input command via kelo_tulip interface.

        :param input_cmd: Control input command of shape [v0, w0]
        :type input_cmd: np.array
        '''
        cmd_twist = Twist()
        cmd_twist.linear.x = input_cmd[0]
        cmd_twist.angular.z = input_cmd[1]
        self.cmd_vel_pub.publish(cmd_twist)

    def get_kelo_status(self, status):
        '''Callback for platform_driver/status topic.

        .. todo::
          Actually implement the reading of KELO status, instead of the dummy
          here.        '''
        self.platform_driver_status = True

    def wait_for_kelo_status(self):
        '''Block further operations until contact is established with KELO
        interface.

        .. todo::
          Actually implement the reading of KELO status, instead of the dummy
          here.
        '''
        rospy.loginfo('CTRL - Waiting for platform driver status')
        while not self.platform_driver_status and not rospy.is_shutdown():
            # TODO: REMOVE THIS! must be set to True via topic callback
            self.platform_driver_status = True
            self.ctrl_rate.sleep()

        rospy.loginfo('CTRL - Received platform driver status')

    ##################
    # Identification #
    ##################
    def identify_velocity(self):
        '''Perform a series of predefined input commands for the identification
        of the velocity response. In- and output are saved to a .mat-file after
        the experiment.
        '''

        # Create input signal
        tlen = 10.  # s
        step = 2*np.pi / 5
        vref = [0 for _ in range(int(tlen/self.sample_time))]
        omegaref = [step for _ in range(int(tlen/self.sample_time))]

        v_wheel_input_rec = [None for _ in range(len(vref))]
        omega_input_rec = [None for _ in range(len(vref))]
        pos_x_rec = [None for _ in range(len(vref))]
        pos_y_rec = [None for _ in range(len(vref))]
        theta_rec = [None for _ in range(len(vref))]

        time_rec = [None for _ in range(len(vref))]

        input_cmd = np.zeros(2)

        i = 0

        while not (i >= len(vref) or rospy.is_shutdown() or self.state_killed):

            print(i, vref[i], omegaref[i])
            input_cmd[0] = vref[i]
            input_cmd[1] = omegaref[i]
            print(input_cmd)
            self.send_input_cmd(input_cmd)

            # Record
            v_wheel_input_rec[i] = vref[i]
            omega_input_rec[i] = omegaref[i]
            pos_x_rec[i] = self.loc_bridge.pose_est.x
            pos_y_rec[i] = self.loc_bridge.pose_est.y
            theta_rec[i] = self.loc_bridge.pose_est.theta
            time_rec[i] = rospy.Time.now().to_sec()

            i += 1
            self.ctrl_rate.sleep()

        for i in range(int(2/self.sample_time)):
            self.safety_brake()
            self.ctrl_rate.sleep()

        # Save data
        if not (self.state_killed or rospy.is_shutdown()):
            data = {}
            data['Description'] = (
                "- Vehicle: AMR with Kelo wheel, " +
                "- Battery: 6S" +
                "- Experiment: Step response " +
                " on linear velocity for 5s." +
                "Measure position and orientation.")

            data['v_wheel'] = v_wheel_input_rec
            data['omega'] = omega_input_rec
            data['pos_x'] = pos_x_rec
            data['pos_y'] = pos_y_rec
            data['theta'] = theta_rec
            data['time'] = time_rec

            from datetime import datetime
            datetimestr = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

            io.savemat(self.record_directory +
                       'recording_velocity_step_' +
                       str(step) + '_' +
                       datetimestr + '.mat', data)
            rospy.loginfo("CTRL - Recording saved.")

    def test_fb_ctrl(self):
        '''Drive backwards along a given trajectory to test if the state
        feedback controller can stabilize and prevent jack-knifing.
        '''
        state_x = self.get_state_x()
        state_x_ref0 = np.array([state_x[0], state_x[1]-0.1, 0, 0, 0])
        state_x_ref = np.array([*state_x_ref0])

        traj = "linear"
        # traj = "circular"

        v = 0.5
        if traj == "linear":
            # Linear trajectory
            cmd_ff = np.array([v, 0.])
            t_end = 8.
            N = int(t_end/self.sample_time)
            t = np.linspace(0, t_end, N+1)

        elif traj == "circular":
            # Circular trajectory
            radius = 1.
            pulsation = abs(v / radius)
            cmd_ff = np.array([v, np.sign(v) * pulsation])
            # t_end = 2*np.pi / pulsation
            t_end = 40.
            N = int(t_end/self.sample_time)
            # t = np.linspace(0, t_end, N+1)
            t = np.arange(0, t_end, self.sample_time)

        i = 0
        while not ((i >= N) or rospy.is_shutdown() or self.state_killed):

            # ff + fb
            self.ctrl_update(cmd_ff, state_x_ref)

            if traj == "linear":
                # Drive on a line
                state_x_ref[0] += cmd_ff[0] * self.sample_time

            elif traj == "circular":
                # Drive on a circle
                state_x_ref[0] = state_x_ref0[0] + \
                    np.sign(v) * radius * np.sin(pulsation*i*self.sample_time)
                state_x_ref[1] = state_x_ref0[1] + \
                    radius * (1 - np.cos(pulsation*i*self.sample_time))
                state_x_ref[2] = np.sign(v) * pulsation*i*self.sample_time

            setpoint = PointStamped()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.header.frame_id = "map"
            setpoint.point.x = state_x_ref[0]
            setpoint.point.y = state_x_ref[1]
            self.setpoint_pub.publish(setpoint)

            i += 1
            self.ctrl_rate.sleep()

    def test_wheel_slip(self):
        '''
        Apply a velocity reference step from v = 0 to v = value and measure
        odometry, position. Compare them to detect wheel slip.

        '''
        # Create input signal
        tlen = 1.5  # s
        N = int(tlen/self.sample_time)
        print('N', N)
        vstep = 1.2  # m/s
        vref = [vstep for _ in range(N)] +\
               [0 for _ in range(N)]

        i = 0
        while not ((i >= len(vref)) or
                   rospy.is_shutdown() or self.state_killed):
            print(i)
            input_cmd = np.array([vref[i], 0])
            self.send_input_cmd(input_cmd)

            i += 1
            self.ctrl_rate.sleep()

    #######################
    # Encoder calibration #
    #######################
    def drive_fwd_stop(self):
        '''Drive forward for a pre-specified distance to align the trailer with
        the AMR. Next, set the encoder angle offset to define the zero point.
        '''
        # Create input signal
        tlen = 3.  # s
        step = 0.1
        vref = [step for _ in range(int(tlen/self.sample_time))]
        omegaref = [0 for _ in range(int(tlen/self.sample_time))]
        input_cmd = np.zeros(2)

        rospy.loginfo('CTRL - Start angle encoder calibration.' +
                      'Warning: vehicle will move forward')
        rospy.sleep(1.)
        i = 0
        while not (i >= len(vref) or rospy.is_shutdown() or self.state_killed):

            input_cmd[0] = vref[i]
            input_cmd[1] = omegaref[i]

            self.send_input_cmd(input_cmd)

            i += 1
            self.ctrl_rate.sleep()

        for i in range(int(2/self.sample_time)):
            self.safety_brake()
            self.ctrl_rate.sleep()

        self.encoder_bridge.reset_encoder()

    ####################
    # Helper functions #
    ####################
    def fsm_handshake(self, _):
        '''Respond the service call with an empty message to inform the caller
        (FSM) that controller is running.

        :param _: Placeholder for empty topic callback.
        :type _: std_msgs.msg.Empty

        :return: success
        :rtype: std_srvs.srv.Trigger (response)
        '''
        return {"success": True, "message": ""}

    def switch_state(self, state):
        '''Switches state according to the fsm state as received by
        fsm node.

        :param state: The state commanded by FSM.
        :type state: str
        '''
        if not (state.data == self.state):
            self.state = state.data
            self.state_changed = True
            # If new state received before old one is finished,
            # kill current state.
            if self.executing_state:
                self.state_killed = True
        else:
            rospy.loginfo('CTRL - Controller already in the correct state!')
            if not self.executing_state:
                # Allows to proceed to next state after same task is requested
                # twice and state was finished in between.
                self.state_finish_pub.publish(Empty())

        # When going to standby, remove markers in Rviz from previous task.
        if state.data == "standby":
            self.rviz_handler.reset_traj_markers()

    def get_state_x(self):
        '''Return the current state

        :return: the current state x = [px1, py1, theta1, theta0, omega0]
        :rtype: np.array
        '''
        return np.array([self.loc_bridge.pose_est.x,
                         self.loc_bridge.pose_est.y,
                         self.loc_bridge.pose_est.theta,
                         self.loc_bridge.pose_est.theta +
                         self.encoder_bridge.angle_meas,
                         self.loc_bridge.yaw_vel_est +
                         self.encoder_bridge.enc_speed_est])

    def check_goal_reached(self, goal_state, current_state, pos_tol=None,
                           orient_tol=None):
        '''Determines whether goal is reached.
        Used for:

            1. checking whether position setpoint is reached
            2. checking whether goal for motionplanner is reached

        :param goal_state: The goal state.
        :type goal_state: np.array([px1, py1, theta1, theta0])

        :param current_state: The current state.
        :type current_state: np.array([px1, py1, theta1, theta0])

        :param pos_tol: tolerance on position to declare goal reached
        :type pos_tol: float

        :param orient_tol: tolerance on orientation to declare goal reached
        :type orient_tol: float

        :return: goal_reached - whether or not goal is reached
        :rtype: bool
        '''
        if not pos_tol:
            tol = self.pos_nrm_tol
            orient_tol = self.orient_tol

        pos_nrm = np.linalg.norm(current_state[0:2] - goal_state[0:2])
        theta1_diff = abs((current_state[2] - goal_state[2]) % (2*np.pi))
        theta1_diff = ((theta1_diff - np.pi) % (2*np.pi)) - np.pi

        goal_reached = (pos_nrm < pos_tol and theta1_diff < orient_tol)

        return goal_reached

    def mp_visualize(self):
        '''Plot in rviz the current position, velocity, planned position and
        velocity.
        '''
        # Publish pose to append to travelled path in rviz.
        self.rviz_handler.publish_real(self.loc_bridge.pose_est.x,
                                       self.loc_bridge.pose_est.y,
                                       0)

        # Visualize current reference velocity calculated by mp in rviz.
        pos = PointStamped()
        pos.header.frame_id = "map"
        pos.point.x = self.mp_traj['px1'][self.mp_index]
        pos.point.y = self.mp_traj['py1'][self.mp_index]

        self.trigger_point_pub.publish(
                    PointStamped(header=Header(stamp=rospy.Time.now(),
                                 frame_id="map"),
                                 point=Point(*self.mp_trigger.x_t0[0:2], 0)))

    def visualize_poses(self):
        '''Visualize truck and trailer poses in Rviz.
        '''
        truck_pose, trailer_pose = self.get_truck_trailer_poses()
        self.rviz_handler.publish_truck_trailer_poses(truck_pose, trailer_pose)

    def visualize_velocities(self, v1_fb, v0_ff, v0_fb):
        '''Visualize velocities (feedforward, feedback) for trailer and truck.

        :param v1_fb: Trailer feedback velocity.
        :type v1_fb: np.array([vx, vy])

        :param v0_ff: Truck feedforward velocity.
        :type v0_ff: np.array([vx, vy])

        :param v0_fb: Truck feedback velocity.
        :type v0_fb: np.array([vx, vy])
        '''
        truck_pose, trailer_pose = self.get_truck_trailer_poses()
        self.rviz_handler.publish_trailer_velocities(trailer_pose, v1_fb)
        self.rviz_handler.publish_truck_velocities(truck_pose, v0_ff, v0_fb)

        t = rospy.Time.now()

        v0_fb_vector = Vector3Stamped()
        v0_fb_vector.vector = Vector3(*v0_fb, 0)
        v0_fb_vector.header.stamp = t
        v0_ff_vector = Vector3Stamped()
        v0_ff_vector.vector = Vector3(*v0_ff, 0)
        v0_ff_vector.header.stamp = t
        v1_fb_vector = Vector3Stamped()
        v1_fb_vector.vector = Vector3(*v1_fb, 0)
        v1_fb_vector.header.stamp = t

        self.truck_fb_vel_pub.publish(v0_fb_vector)
        self.truck_ff_vel_pub.publish(v0_ff_vector)
        self.trailer_fb_vel_pub.publish(v1_fb_vector)

    def get_truck_trailer_poses(self):
        '''From the current measurements, get the current pose of truck and
        trailer.

        :return: (truck_pose, trailer_pose)

            - truck_pose (geometry_msgs.msg.Pose2D) - the truck pose
            - trailer_pose (geometry_msgs.msg.Pose2D) - the trailer pose
        '''
        trailer_pose = self.loc_bridge.pose_est
        truck_pose = helpers.get_truck_pose(trailer_pose,
                                            self.encoder_bridge.angle_meas,
                                            self.veh_params)
        return truck_pose, trailer_pose

    def dont(self):
        '''Easter egg :p
        '''
        self.safety_brake()
        self.mp_init = True


if __name__ == '__main__':
    controller = Controller()
    controller.start()
