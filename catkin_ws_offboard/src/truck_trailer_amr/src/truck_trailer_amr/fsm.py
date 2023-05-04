#!/usr/bin/env python3

from std_msgs.msg import String, Empty, Bool
from truck_trailer_amr.msg import Trajectories

from std_srvs.srv import Trigger

import numpy as np
import rospy


class FSM:
    '''Finite State Machine for discrete control: commands the switching of
    continuous controller behavior (controller.py) by selecting and
    broadcasting subsequent states corresponding to a user-specified task.
    '''

    def __init__(self):
        '''
        Initialization of FSM object.
        '''
        rospy.init_node('fsm')
        rospy.loginfo("FSM  - Initialization started")

        self.state = "initialization"
        self.state_sequence = []
        self.change_state = False
        self.new_task = False
        self.state_finish = False
        self.mp_standby = False
        self.task_dict = {
            "standby": [],
            "emergency": ["emergency"],
            "stop": ["brake"],
            "calibrate angle encoder":
                ["encoder calib maneuver"],
            "execute feedforward traj":
                ["ff traj drive"],
            "point to point":
                ["mp standby", "p2p"],
            "warehouse drive multistage":
                ["mp standby", "warehouse drive multistage"],
            "identify velocity": ["identify velocity"],
            "test feedback control": ["test feedback control"],
            "test wheel slip": ["test wheel slip"],
            }

        self.navigation_states = {"mp standby", "p2p",
                                  "warehouse drive multistage"}

        # Params
        self.monitor_rate = rospy.Rate(rospy.get_param(
            '/fsm/monitor_rate', 0.))

        # Topics
        self.fsm_state = rospy.Publisher(
            '/fsm/state', String, queue_size=1)
        self.fsm_state.publish("initialization")

        rospy.Subscriber(
            '/fsm/task', String, self.switch_task)
        rospy.Subscriber(
            '/fsm/state_proceed', Empty, self.switch_state)
        rospy.Subscriber(
            '/fsm/trigger_mp_drive', Empty, self.trigger_mp_drive)
        rospy.Subscriber(
            '/controller/state_finish', Empty, self.ctrl_state_finish)
        rospy.Subscriber('/fsm/update_rosparams', Empty, self.config_mp)

        # Services
        self.service_timeout = 40.
        self.controller_handshake_proxy = rospy.ServiceProxy(
            "/controller/fsm_handshake", Trigger)
        self.config_mp_proxy = rospy.ServiceProxy(
            "/motionplanner/config_motionplanner", Trigger)

    def controller_handshake(self):
        '''Call the controller handshake service to make sure controller is
        running properly.
        '''
        try:
            rospy.wait_for_service(
                "/controller/fsm_handshake", timeout=self.service_timeout)
            success = self.controller_handshake_proxy()
        except Exception as e:
            rospy.logerr('FSM - Controller handshake failed. ' +
                         'Are you sure CTRL is running? Error message:\n' +
                         '%s' % e)
            success = False
        return success

    def config_mp(self, _=None):
        '''Configures the motionplanner over ConfigMotionplanner Service.

        :param _: Placeholder for empty topic callback
        :type _: std_msgs.msg.Empty

        :return: config_response - succes of mp configuration
        :rtype: bool
        '''
        # =================
        # BYPASS MP CONFIG
        # return True
        # =================
        try:
            rospy.wait_for_service("/motionplanner/config_motionplanner",
                                   timeout=self.service_timeout)
            config_response = self.config_mp_proxy()
        except Exception as e:
            rospy.logerr(
                        'FSM  - MP configuration service call failed:\n%s' % e)
            return False

        return config_response

    def start(self):
        '''
        Starts running of FSM. Runs along the state sequence, sends
        out the current state and returns to the standby state when task is
        completed.
        '''
        # Configure and check MP and CTRL.
        if not (self.controller_handshake() and self.config_mp()):
            rospy.logerr('FSM  - Initialization failed')
            return

        rospy.loginfo('FSM  - *** Initialization complete! ***')
        # Go to standby when initialization is complete
        self.new_task = True

        while not rospy.is_shutdown():
            if self.new_task:
                self.new_task = False
                self.change_state = False
                self.state_finish = False

                # Run over sequence of states corresponding to current task.
                for state in self.state_sequence:
                    self.state = state
                    rospy.loginfo(
                        'FSM  - State changed to: ' + self.state)
                    self.fsm_state.publish(state)

                    if state == "mp standby":
                        self.state_finish = True

                    # mp should return to its own standby status unless
                    # the state change button has been pressed.
                    if self.state in self.navigation_states:
                        self.mp_standby = True
                    else:
                        self.mp_standby = False

                    task_final_state = (self.state == self.state_sequence[-1])
                    # Check if previous state is finished and if allowed to
                    # switch state based on controller input.
                    while not ((self.state_finish and (
                                self.change_state or task_final_state)) or
                               self.new_task or rospy.is_shutdown()):
                        # Remaining in state. Allow state action to continue.
                        self.monitor_rate.sleep()

                    self.change_state = False
                    self.state_finish = False

                    leave_mp = (
                        self.state == "mp standby" and not self.mp_standby)
                    # User forces leaving mp with button or other new task
                    # received --> leave the for loop for the current task.
                    if (leave_mp or self.new_task):
                        break

                # Make sure that mp task is repeated until force quit.
                if self.mp_standby:
                    self.new_task = True
                    self.state_finish = True
                # Only publish standby state when task is finished.
                # Except for repetitive tasks (back to first state in task).
                if not self.new_task:
                    self.state = "standby"
                    self.fsm_state.publish("standby")
                    rospy.loginfo('FSM  - State changed to: ' + "standby")

            self.monitor_rate.sleep()

    def switch_task(self, task):
        '''Reads out the task topic and switches to the desired task.
        '''
        if task.data not in self.task_dict:
            rospy.logerr('FSM  - Not a valid task, \n' +
                         'fsm will remain in standby state.')

        self.state_sequence = self.task_dict.get(task.data, [])
        self.new_task = True
        rospy.loginfo('FSM  - Received a new task: ' + task.data)

    def switch_state(self, _):
        '''When proceed is pressed changes change_state variable
        to true to allow fsm to switch states in state sequence.
        '''
        if not self.state_finish:
            rospy.loginfo("FSM  - State action not finished yet")
            return

        if self.state_finish and (
                self.state not in {"standby", "initialization"}):

            self.change_state = True
            if self.state == "mp standby":
                self.mp_standby = False
                self.new_task = False
            rospy.loginfo('FSM  - Switching to next state ')

    def trigger_mp_drive(self, _):
        '''Triggers transition from 'mp standby' state to next state.
        '''
        if self.state == "mp standby":
            self.change_state = True

    ####################
    # Helper functions #
    ####################
    def ctrl_state_finish(self, _):
        '''Checks whether controller has finished the current state.

        :param _: Placeholder for empty topic callback
        :type _: std_msgs.msg.Empty
        '''
        self.state_finish = True


if __name__ == '__main__':
    fsm = FSM()
    fsm.start()
