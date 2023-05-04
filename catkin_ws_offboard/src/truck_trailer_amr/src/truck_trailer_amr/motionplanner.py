#!/usr/bin/env python3

from trailer_agv_multistage.msg import Trigger_mp, Trajectories
from std_msgs.msg import Empty

from std_srvs.srv import Trigger

import trailer_agv_multistage.helper_funcs_mp as helpers
from casadi import *
import numpy as np
from rockit import *
import rospy
import yaml


class MotionPlanner(object):

    ##################
    # Initialization #
    ##################
    def __init__(self, standalone=False, param_path=None):
        '''Initializes motion planner.
        Standalone is for when MotionPlanner is called outside of ROS context
        to generate the solve_ocp() CasADi function and save it. In that case,
        the parameters must not be loaded from the ROS parameter server, but
        directly from the .yaml files. Give the path for the parameters through
        param_path (string).
        '''
        # Constants
        self.service_timeout = 10.
        self.standalone = standalone

        if not standalone:
            rospy.init_node('motionplanner')
        self._init_variables()
        self._init_params(param_path)
        if not standalone:
            self._init_msg_srv()

    def _init_variables(self):
        '''Initialize variables.
        '''
        pass

    def _init_params(self, param_path=None):
        '''Initialize motion planner parameters:
                - Optimization problem parameters
                - Vehicle geometry and dynamics parameters
                - Environment geometry parameters
        '''
        try:
            if self.standalone:
                file_mp = open(param_path + "motionplanner_config.yaml")
                file_ctrl = open(param_path + "controller_config.yaml")
                mp_params = yaml.load(file_mp, Loader=yaml.FullLoader)
                ctrl_params = yaml.load(file_ctrl, Loader=yaml.FullLoader)

            else:
                ctrl_params = rospy.get_param('/controller')
                mp_params = rospy.get_param('/motionplanner')

            #  - Optimization problem parameters
            self.opt = mp_params['opti']
            self.opt['Ts'] = ctrl_params['sample_time']
            self.opt["mp_update_index"] = ctrl_params['mp_update_index']

            #  - Vehicle geometry and dynamics parameters
            self.veh = mp_params['vehicles']
            #  - Environment geometry parameters
            self.env = mp_params['environment']

            self.show_comp_stats = mp_params['show_computation_time_stats']

            rospy.loginfo("MP   - Parameters loaded")

        except Exception as e:
            rospy.logerr("MP    - Parameter(s) missing, " +
                         "check the config yaml file.\n %s" % e)

    def _init_msg_srv(self):
        '''Initialize topics and services.
        '''
        # Topics
        self._mp_result_topic = rospy.Publisher(
            '/motionplanner/result', Trajectories, queue_size=1)

        rospy.Subscriber(
            '/motionplanner/trigger', Trigger_mp, self.compute_MPC)
        rospy.Subscriber('/fsm/update_rosparams', Empty, self._init_params)

        # Services
        self.configure_srv = rospy.Service(
            "/motionplanner/config_motionplanner", Trigger, self.configure)

    def start(self):
        '''Starts the motionplanner by initializing the motionplanner ROS-node.
        '''

        rospy.spin()

    #########################
    # MPC setup and solving #
    #########################
    def configure(self, _=None):
        '''Configures the motionplanner. Creates Point2point problem.
        Creates (and saves) a CasADi function for this problem.
        '''
        mp_configured = False

        # Definition of the OCP
        ocp = Ocp(T=FreeTime(20.0))
        # Trailer model
        # States
        x = ocp.state(4)
        px1 = x[0]
        py1 = x[1]
        theta1 = x[2]
        theta0 = x[3]

        # Controls & Control derivatives
        v0 = ocp.control(1, order=1)
        omega0 = ocp.control(1, order=1)
        u = vertcat(v0, omega0)

        # Dynamics
        L0 = self.veh['truck']['L']
        M0 = self.veh['truck']['M']
        W0 = self.veh['truck']['W']
        L1 = self.veh['trailer1']['L']
        M1 = self.veh['trailer1']['M']
        W1 = self.veh['trailer1']['W']

        # Geometric relations
        beta01 = theta0 - theta1
        v1 = v0*cos(beta01) + M0*sin(beta01)*omega0
        omega1 = v0/L1*sin(beta01) - M0/L1*cos(beta01)*omega0

        rhs = vertcat(v1*cos(theta1), v1*sin(theta1), omega1, omega0)

        ocp.set_der(x, rhs)
        udot = ocp.der(u)
        dv0 = udot[0]
        domega0 = udot[1]
        # ddelta0 = ocp.der(delta0)

        # Initial constraints
        x_t0 = ocp.parameter(4)
        u_t0 = ocp.parameter(2)
        ocp.subject_to(ocp.at_t0(x) == x_t0)
        ocp.subject_to(ocp.at_t0(u) == u_t0)  # Necessary for order=1

        # Terminal constraint
        x_tf = ocp.parameter(4)
        u_tf = ocp.parameter(2)
        ocp.subject_to(ocp.at_tf(x) == x_tf)
        ocp.subject_to(ocp.at_tf(u) == u_tf)

        # Path constraints
        ocp.subject_to(self.veh['truck']['v0_min'] <= (
                       v0 <= self.veh['truck']['v0_max']),
                       include_first=False)
        ocp.subject_to(self.veh['truck']['dv0_min'] <= (
                       dv0 <= self.veh['truck']['dv0_max']),
                       include_first=False)
        ocp.subject_to(-self.veh['truck']['domega0_max'] <= (
                       domega0 <= self.veh['truck']['domega0_max']),
                       include_first=False)
        ocp.subject_to(-self.veh['beta01_max'] <= (
                       beta01 <= self.veh['beta01_max']),
                       include_first=False)

        # Minimal time
        ocp.add_objective(ocp.T)
        # ocp.add_objective(ocp.integral(u.T @ diag(self.opt['R']) @ u))
        # ocp.add_objective(ocp.integral(udot.T @ diag(self.opt['Rdot']) @ udot))

        ocp.method(MultipleShooting(N=self.opt['N'], M=self.opt['M'],
                                    intg='rk'))

        opts = {"expand": True,
                "verbose": False,
                # "print_time": True,
                "error_on_fail": True,
                "ipopt": {"linear_solver": "ma27",
                          "max_iter": 1000,
                          # 'print_level': 5,
                          'sb': 'yes',  # Suppress IPOPT banner
                          'tol': 1e-4,
                          # 'warm_start_init_point': 'yes',
                          # 'warm_start_bound_push': 1e-8,
                          # 'warm_start_mult_bound_push': 1e-8,
                          # 'mu_init': 1e-5,
                          # 'hessian_approximation': 'limited-memory'
                          }}

        if self.standalone:
            opts["print_time"] = True
            opts["ipopt"]["print_level"] = 5
        else:
            opts["print_time"] = False
            opts["ipopt"]["print_level"] = 0

        # opts = {}
        ocp.solver("ipopt", opts)

        # -----
        # SOLVE
        # -----

        # Set parameter dummy values
        ocp.set_value(x_t0, vertcat(0, 0, 0, 0))
        ocp.set_value(x_tf, vertcat(0, 0, 0, 0))
        ocp.set_value(u_t0, vertcat(0, 0))
        ocp.set_value(u_tf, vertcat(0, 0))

        # Create sampler
        self.sampler = ocp.sampler([x, u])

        # Creating a casadi function to enable MPC updates.
        self.solve_ocp = ocp.to_function(
            'solve_ocp',
            [ocp.value(u_t0), ocp.value(u_tf),
             ocp.value(x_t0), ocp.value(x_tf)],
            [ocp.sample(x, grid='control')[0], ocp.sample(x, grid='control')[1],
            ocp.value(ocp.T), ocp.gist])

        if self.standalone:
            self.ocp = ocp
            self.x_t0 = x_t0
            self.x_tf = x_tf
            self.u_t0 = u_t0
            self.u_tf = u_tf

        mp_configured = True
        rospy.loginfo("MP   - Motionplanner configured")

        return {"success": mp_configured, "message": ''}

    def compute_MPC(self, cmd):
        '''Callback of trigger topic - result of fire motion planner.
        Start an MPC update computation.

        Args:
            cmd : contains data sent over Trigger topic.
        '''
        t0 = rospy.Time.now()
        # Preprocess.
        solve_ocp_args = self.solve_ocp_preproc(cmd)
        t1 = rospy.Time.now()
        # Process.
        result = self.MPC_update(solve_ocp_args)
        t2 = rospy.Time.now()

        # Artifically slow down updates for sensitivity analysis.
        # rospy.sleep(0.2)

        # Postprocess.
        result = self.solve_ocp_postproc(result)
        t3 = rospy.Time.now()

        if self.show_comp_stats:
            rospy.loginfo(
                'MP   - Comp stats:'
                '\n Prepr: ' + str(round((t1-t0).to_sec(), 3)) + '; ' +
                '\n Proc: ' + str(round((t2-t1).to_sec(), 3)) + '; ' +
                '\n Postpr: ' + str(round((t3-t2).to_sec(), 3)))

        # Publish the result to controller.
        self._mp_result_topic.publish(result)

    def solve_ocp_preproc(self, cmd):
        '''Shape the input arguments for the MPC update to the required format.

        Args:
            cmd: Trigger_mp().
        '''
        u_t0 = vertcat(cmd.u_t0)
        u_tf = vertcat(cmd.u_tf)
        x_t0 = vertcat(cmd.x_t0)
        x_tf = vertcat(cmd.x_tf)

        solve_ocp_args = {'init_control': u_t0,
                          'terminal_control': u_tf,
                          'init_state': x_t0,
                          'terminal_state': x_tf}

        # Debug failure to find solution.
        # -------------------------------
        # print("Init and terminal conditions")
        # print('x_t0', x_t0)
        # print('x_tf', x_tf)
        # print('u_t0', u_t0)
        # print('u_tf', u_tf)

        return solve_ocp_args

    def MPC_update(self, solve_ocp_args):
        '''MPC update computation.
        Publishes trajectories resulting from computations.

        Args:
            solve_ocp_args: state, goal, batt_volt, solve_ocp function as
            produced by solve_ocp_preproc.
        '''
        calc_succeeded = False
        try:
            # [Tsol, gist] = self.solve_ocp(
            [tsol_coarse, xsol_coarse, Tsol, gist] = self.solve_ocp(
                solve_ocp_args["init_control"],
                solve_ocp_args["terminal_control"],
                solve_ocp_args["init_state"],
                solve_ocp_args["terminal_state"])

            calc_succeeded = True

        except Exception as e:
            rospy.logwarn(
                'MP   - No solution found in MPC step. Error message:\n' +
                str(e) +
                '\n Boundary conditions were\n' +
                "u0: " + str(solve_ocp_args["init_control"]) + "\n" +
                "uf: " + str(solve_ocp_args["terminal_control"]) + "\n" +
                "x0: " + str(solve_ocp_args["init_state"]) + "\n" +
                "xf: " + str(solve_ocp_args["terminal_state"]))
            calc_succeeded = False
            xsol_coarse = DM.zeros(4)
            Tsol = 0.
            gist = 0.

        if self.show_comp_stats:
            rospy.loginfo("MP   - Solution of Time-to-goal: " +
                          str(Tsol) + " s")

        result = {"tsol": tsol_coarse,
                  "xsol": xsol_coarse.full(),
                  "Tsol": float(Tsol),
                  "gist": gist,
                  "success": calc_succeeded}

        return result

    def solve_ocp_postproc(self, sol):
        '''Shape the solution to the required format.
        '''
        if not sol["success"]:

            xsol_fine = np.zeros((4, 100))
            usol_fine = np.zeros((2, 100))
            stage_indices = [0, 0]

        else:

            # Control rate sampling - only finely sample the part that will
            # actually be applied!
            t = np.arange(0., self.opt["Ts"] * self.opt["mp_update_index"],
                          self.opt["Ts"])

            xsol_fine, usol_fine = self.sampler(sol['gist'], t)
            # Convert solution to standard type (lists).
            [xsol_fine, usol_fine] = helpers.solution_to_lists(
                                                    [xsol_fine.T, usol_fine.T])

        tsol_coarse = sol["tsol"]
        xsol_coarse = sol["xsol"]
        # Dummy for test:
        stage_indices = [10, 20, 20]

        result = Trajectories(
            t_coarse=tsol_coarse,
            v0=usol_fine[0],
            omega0=usol_fine[1],
            px1_coarse=xsol_coarse[0],
            py1_coarse=xsol_coarse[1],
            theta1_coarse=xsol_coarse[2],
            theta0_coarse=xsol_coarse[3],
            px1=xsol_fine[0],
            py1=xsol_fine[1],
            theta1=xsol_fine[2],
            theta0=xsol_fine[3],
            stage_indices=stage_indices,
            success=sol["success"])

        return result


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
