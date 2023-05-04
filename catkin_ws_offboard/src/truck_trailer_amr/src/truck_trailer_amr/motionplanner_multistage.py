#!/usr/bin/env python3

from truck_trailer_amr.msg import Trigger_mp, Trajectories, StageTiming
import truck_trailer_amr.helper_funcs_mp as helpers

from std_msgs.msg import Empty, Float32

from std_srvs.srv import Trigger

import rockit as r
import casadi as c
import numpy as np
import rospy
import yaml
import time


class MotionPlanner(object):
    '''A class for optimal motion planning with Rockit. This class is a ROS
    node that communicates with controller, to enable asynchronous motion
    planning updates and additional low-level feedback control.
    '''
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

        :param standalone: If true, an instance of the class is constructed
                           without a rosnode, such that there is no need of a
                           roscore.
        :type standalone: bool

        :param param_path: The path to the config file directory. Only used in
                           standalone mode.
        :type param_path: str
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
        self.old_corr = np.zeros((3, 12))

    def _init_params(self, param_path=None):
        '''Initialize motion planner parameters:
                - Optimization problem parameters
                - Vehicle geometry and dynamics parameters
                - Environment geometry parameters
                - Statistical output

        :param param_path: The path to the config file directory. Only used in
                           standalone mode.
        :type param_path: str
        '''
        try:
            if self.standalone:
                file_mp = open(param_path +
                               "motionplanner_multistage_config.yaml")
                mp_params = yaml.load(file_mp, Loader=yaml.FullLoader)
                file_mp.close()

                file_ctrl = open(param_path + "controller_config_sim.yaml")
                ctrl_params = yaml.load(file_ctrl, Loader=yaml.FullLoader)
                file_ctrl.close()
            else:
                mp_params = rospy.get_param('/motionplanner')
                ctrl_params = rospy.get_param('/controller')

            # Optimization problem parameters
            self.opt = mp_params['opti']
            self.opt['Ts'] = ctrl_params['sample_time']
            self.opt['mp_update_index'] = ctrl_params['mp_update_index']

            # Vehicle geometry and dynamics parameters
            self.veh = mp_params['vehicles']

            # Environment geometry parameters
            self.env = mp_params['environment']

            # Statistical output
            self.show_comp_stats = mp_params['show_computation_time_stats']

            rospy.loginfo("MP   - Parameters loaded")

        except Exception as e:
            rospy.logerr("MP   - Parameter(s) missing, " +
                         "check the config yaml file.\n %s" % e)

    def _init_msg_srv(self):
        '''Initialize topics and services.
        '''
        # Topics
        self._mp_result_topic = rospy.Publisher(
            '/motionplanner/result', Trajectories, queue_size=1)

        self._mp_calc_time_topic = rospy.Publisher(
            '/motionplanner/calc_time', Float32, queue_size=1)

        self._mp_stage_time_topic = rospy.Publisher(
            '/motionplanner/stage_time', StageTiming, queue_size=1)

        rospy.Subscriber(
            '/motionplanner/trigger', Trigger_mp, self.compute_MPC)
        rospy.Subscriber(
            '/fsm/update_rosparams', Empty, self._init_params)

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
    def create_stage(self, ocp, t0, T, N, M, include_first=True):
        '''Define a single stage of the OCP.

        :param ocp: optimal control problem
        :type ocp: rockit.Ocp

        :param t0: (free) starting time
        :type t0: float OR rockit.FreeTime

        :param T: (free) time horizon
        :type T: float OR rockit.FreeTime

        :param N: number of control intervals
        :type N: int

        :param M: number of integration steps per control interval
        :type M: int

        :param include_first: enforces constraint also at t0
        :type include_first: bool

        :return: (stage, x, u, udot, w_truck, w_trailer)

                 - stage (rockit.Stage) - ocp stage
                 - x (casadi.MX) - state vector
                 - u (casadi.MX) - control vector
                 - udot (casadi.MX) - derivative of control vector
                 - w_truck (casadi.MX) - parametric corridor vectors for truck
                 - w_trailer (casadi.MX) - parametric corridor vectors for
                   trailer
        '''
        # Vehicle geometry and parameters
        L0 = self.veh['truck']['L']
        M0 = self.veh['truck']['M']
        W0 = self.veh['truck']['W']
        L1 = self.veh['trailer1']['L']
        M1 = self.veh['trailer1']['M']
        W1 = self.veh['trailer1']['W']

        truck_sd = self.veh['truck']['safety_dist']
        truck_sd_w = self.veh['truck']['sd_weight']
        trailer_sd = self.veh['trailer1']['safety_dist']
        trailer_sd_w = self.veh['trailer1']['sd_weight']

        # Create stage with minimal time horizon of Ts
        stage = ocp.stage(t0=t0, T=T)
        stage.subject_to(stage.T >= self.opt['Ts'])

        # Trailer model
        # State x:
        #   px1, py1: center point of trailer axle
        #   px0, py0: center point of truck axle
        #   theta1: orientation of the trailer
        #   theta0: orientation of the truck
        x = stage.state(4)
        px1 = x[0]
        py1 = x[1]
        theta1 = x[2]
        theta0 = x[3]

        px0 = px1 + L1*c.cos(theta1) + M0*c.cos(theta0)
        py0 = py1 + L1*c.sin(theta1) + M0*c.sin(theta0)

        # Control u:
        #   v0: longitudinal velocity of the truck
        #   omega0: rotational velocity of the truck
        v0 = stage.control(1, order=1)
        omega0 = stage.control(1, order=1)
        u = c.vertcat(v0, omega0)

        # Geometric relations
        beta01 = theta0 - theta1
        v1 = v0*c.cos(beta01) + M0*c.sin(beta01)*omega0
        omega1 = v0/L1*c.sin(beta01) - M0/L1*c.cos(beta01)*omega0

        # ODE
        rhs = c.vertcat(v1*c.cos(theta1), v1*c.sin(theta1), omega1, omega0)
        stage.set_der(x, rhs)

        # Derivatives of control signals
        udot = stage.der(u)
        dv0 = udot[0]
        domega0 = udot[1]

        # Path constraints
        stage.subject_to(self.veh['truck']['v0_min'] <= (
                         v0 <= self.veh['truck']['v0_max']),
                         include_first=include_first)
        stage.subject_to(self.veh['truck']['dv0_min'] <= (
                         dv0 <= self.veh['truck']['dv0_max']),
                         include_first=include_first)
        stage.subject_to(-self.veh['truck']['domega0_max'] <= (
                         domega0 <= self.veh['truck']['domega0_max']),
                         include_first=include_first)
        stage.subject_to(-self.veh['beta01_max'] <= (
                         beta01 <= self.veh['beta01_max']),
                         include_first=include_first)

        # Parametric corridor constraints based on vehicle vertices
        truck_vertices = helpers.get_vehicle_vertices(
                                          px0, py0, theta0, W0/2, W0/2, L0, M0)
        trailer_vertices = helpers.get_vehicle_vertices(
                                          px1, py1, theta1, W1/2, W1/2, L1, M1)
        w_truck = stage.parameter(3, 4)
        w_trailer = stage.parameter(3, 4)

        # Slack variables to allow introduction of a safety distance
        truck_slack = stage.variable(grid='control')
        trailer_slack = stage.variable(grid='control')
        stage.subject_to(truck_slack <= 0.)
        stage.subject_to(trailer_slack <= 0.)

        for i in range(truck_vertices.size(2)):
            vertex = truck_vertices[:, i]
            phom = c.vertcat(vertex[0], vertex[1], 1)
            stage.subject_to(w_truck.T @ phom <= truck_slack,
                             grid='control', include_first=include_first)
        for i in range(trailer_vertices.size(2)):
            vertex = trailer_vertices[:, i]
            phom = c.vertcat(vertex[0], vertex[1], 1)
            stage.subject_to(w_trailer.T @ phom <= trailer_slack,
                             grid='control', include_first=include_first)

        # Solution method
        stage.method(r.MultipleShooting(N=N, M=M, intg='rk'))

        # Minimal time
        stage.add_objective(stage.T)

        # Optimal distance to corridors (perpendicular safety distance)
        stage.add_objective(
            truck_sd_w*stage.integral((truck_sd + truck_slack)**2))
        stage.add_objective(
            trailer_sd_w*stage.integral((trailer_sd + trailer_slack)**2))

        # Regularization
        # stage.add_objective(stage.integral(u.T @ diag(self.opt['R']) @ u))
        # stage.add_objective(
        #     stage.integral(udot.T @ diag(self.opt['Rdot']) @ udot))

        return stage, x, u, udot, w_truck, w_trailer

    def stitch_stages(self, ocp, stage_1, stage_2, stage_1_xudu, stage_2_xudu):
        '''Stitches states, controls and derivatives of controls of two stages
        at their connecting boundary.

        :param ocp: optimal control problem
        :type ocp: rockit.Ocp

        :param stage_1: first stage to be connected
        :type stage_1: rockit.Stage

        :param stage_2: second stage to be connected
        :type stage_2: rockit.Stage

        :param stage_1_xudu: states, controls and derivatives of controls
        :type stage_1_xudu: casadi.MX

        :param stage_2_xudu: states, controls and derivatives of controls
        :type stage_2_xudu: casadi.MX
        '''
        for s1_x, s2_x in zip(stage_1_xudu, stage_2_xudu):
            ocp.subject_to(stage_2.at_t0(s2_x) == stage_1.at_tf(s1_x))

    def configure(self, _=None):
        '''Configures the motionplanner. Creates Multistage problem. Creates
        (and saves) a CasADi function for this problem.

        :return: success
        :rtype: std_srvs.srv.Trigger (response)
        '''
        mp_configured = False

        # Definition of the OCP
        ocp = r.Ocp()

        # OCP parameters
        N_1 = self.opt['N_1']
        M_1 = self.opt['M_1']
        T_1 = self.opt['T_1']
        N_2 = self.opt['N_2']
        M_2 = self.opt['M_2']
        T_2 = self.opt['T_2']
        N_3 = self.opt['N_3']
        M_3 = self.opt['M_3']
        T_3 = self.opt['T_3']

        # Stage 1 - Approach
        stage_1, x_1, u_1, udot_1, w0_1, w1_1 = self.create_stage(
                ocp, 0., r.FreeTime(T_1), N_1, M_1, include_first=False)

        # Initial constraint for state and control
        x_t0 = stage_1.parameter(4)
        u_t0 = stage_1.parameter(2)
        stage_1.subject_to(stage_1.at_t0(x_1) == x_t0)
        stage_1.subject_to(stage_1.at_t0(u_1) == u_t0)

        # Stage 2 - Corner
        stage_2, x_2, u_2, udot_2, w0_2, w1_2 = self.create_stage(
                ocp, 0., r.FreeTime(T_2), N_2, M_2, include_first=True)
        self.stitch_stages(
            ocp, stage_1, stage_2, [x_1, u_1, udot_1], [x_2, u_2, udot_2])

        # Stage 3 - Exit
        stage_3, x_3, u_3, udot_3, w0_3, w1_3 = self.create_stage(
                ocp, 0., r.FreeTime(T_3), N_3, M_3, include_first=True)
        self.stitch_stages(
            ocp, stage_2, stage_3, [x_2, u_2, udot_2], [x_3, u_3, udot_3])

        # Terminal constraint for state and control
        x_tf = stage_3.parameter(4)
        u_tf = stage_3.parameter(2)
        stage_3.subject_to(stage_3.at_tf(x_3) == x_tf)
        stage_3.subject_to(stage_3.at_tf(u_3) == u_tf)

        # Limit total time
        ocp.subject_to(stage_1.T + stage_2.T + stage_3.T <= 60.)

        # Define parameters to actively close corridors to avoid numerical
        # instability if stage time horizon tends to go to zero. If closing is
        # active (parameter == 1), stage time horizon is fixed to minimum value
        closed_corridor_2 = stage_2.parameter(1)
        stage_2.subject_to(closed_corridor_2*stage_2.T <= self.opt['Ts'])
        closed_corridor_3 = stage_3.parameter(1)
        stage_3.subject_to(closed_corridor_3*stage_3.T <= self.opt['Ts'])

        # Pick a solver
        opts = {"expand": True,
                "verbose": False,
                "ipopt": {"linear_solver": "ma27",
                          "max_iter": 1000,
                          "sb": "yes",  # Suppress IPOPT banner
                          "tol": 1e-5,
                          # "hessian_approximation": "limited-memory"
                          }}
        if self.standalone:
            opts["print_time"] = True
            opts["ipopt"]["print_level"] = 5
            opts["error_on_fail"] = False
        else:
            opts["print_time"] = False
            opts["ipopt"]["print_level"] = 0
            opts["error_on_fail"]: True

        ocp.solver("ipopt", opts)

        # Set dummy values for all parameters
        stage_1.set_value(x_t0, 0)
        stage_1.set_value(u_t0, 0)
        stage_3.set_value(x_tf, 0)
        stage_3.set_value(u_tf, 0)
        stage_1.set_value(w0_1, 0)
        stage_2.set_value(w0_2, 0)
        stage_3.set_value(w0_3, 0)
        stage_1.set_value(w1_1, 0)
        stage_2.set_value(w1_2, 0)
        stage_3.set_value(w1_3, 0)
        stage_2.set_value(closed_corridor_2, 0)
        stage_3.set_value(closed_corridor_3, 0)

        # Create solution samplers
        sampler_opts = {"jit": True,
                        "compiler": "shell",
                        "jit_temp_suffix": False,
                        "jit_options": {"flags": ['-O3'],
                                        "verbose": False,
                                        }}
        self.samplers = [stage_1.sampler(
                            "sampler1", [x_1, u_1, udot_1], sampler_opts),
                         stage_2.sampler(
                            "sampler2", [x_2, u_2, udot_2], sampler_opts),
                         stage_3.sampler(
                            "sampler3", [x_3, u_3, udot_3], sampler_opts)]

        # Define input of CasADi function by sampling all parameters
        x_t0_s = stage_1.value(x_t0)
        u_t0_s = stage_1.value(u_t0)
        x_tf_s = stage_3.value(x_tf)
        u_tf_s = stage_3.value(u_tf)
        w0_1_s = stage_1.value(w0_1)
        w0_2_s = stage_2.value(w0_2)
        w0_3_s = stage_3.value(w0_3)
        w1_1_s = stage_1.value(w1_1)
        w1_2_s = stage_2.value(w1_2)
        w1_3_s = stage_3.value(w1_3)
        closed_corridor_2_s = stage_2.value(closed_corridor_2)
        closed_corridor_3_s = stage_3.value(closed_corridor_3)
        W0_s = c.horzcat(w0_1_s, w0_2_s, w0_3_s)
        W1_s = c.horzcat(w1_1_s, w1_2_s, w1_3_s)

        # Define output of CasADi function on control grid
        t_s = c.vertcat(stage_1.sample(x_1, grid='control')[0],
                        stage_2.sample(x_2, grid='control')[0],
                        stage_3.sample(x_3, grid='control')[0])
        x_s = c.horzcat(stage_1.sample(x_1, grid='control')[1],
                        stage_2.sample(x_2, grid='control')[1],
                        stage_3.sample(x_3, grid='control')[1])
        u_s = c.horzcat(stage_1.sample(u_1, grid='control')[1],
                        stage_2.sample(u_2, grid='control')[1],
                        stage_3.sample(u_3, grid='control')[1])
        dT1_s = ocp.value(stage_1.T)
        dT2_s = ocp.value(stage_2.T)
        dT3_s = ocp.value(stage_3.T)

        # Create function to provide initial guess for states and controls
        self.initialize_init_guess(ocp, stage_1, stage_2, stage_3,
                                   x_1, x_2, x_3, u_1, u_2, u_3)

        # Create a CasADi function to enable MPC updates
        self.solve_ocp = ocp.to_function(
            'solve_ocp',
            [x_t0_s, u_t0_s, x_tf_s, u_tf_s, W0_s, W1_s,
             closed_corridor_2_s, closed_corridor_3_s,
             ocp._method.opti.x],
            [t_s, x_s, u_s, dT1_s, dT2_s, dT3_s,
             ocp.gist, ocp._method.opti.x])

        if self.standalone:
            self.ocp = ocp
            self.stages = [stage_1, stage_2, stage_3]
            self.x123 = [x_1, x_2, x_3]
            self.u123 = [u_1, u_2, u_3]

            self.x_t0 = x_t0
            self.u_t0 = u_t0
            self.x_tf = x_tf
            self.u_tf = u_tf
            self.w0_1 = w0_1
            self.w0_2 = w0_2
            self.w0_3 = w0_3
            self.w1_1 = w1_1
            self.w1_2 = w1_2
            self.w1_3 = w1_3
            self.closed_corridor_2 = closed_corridor_2
            self.closed_corridor_3 = closed_corridor_3

        mp_configured = True
        rospy.loginfo("MP   - Motionplanner configured")

        return {"success": mp_configured, "message": ''}

    def compute_MPC(self, trigger):
        '''Callback of trigger topic - result of fire motion planner. Start an
        MPC update computation by starting the preprocess, update and
        postprocess sequentially. Calculation times are measured for all these
        steps.

        :param trigger: contains data sent over trigger_mp topic
        :type trigger: truck_trailer_amr.msg.Trigger_mp
        '''
        t0 = time.time()

        # Preprocess
        solve_ocp_args = self.solve_ocp_preproc(trigger)
        t1 = time.time()

        # Process
        result = self.MPC_update(solve_ocp_args)
        t2 = time.time()

        # Postprocess
        result = self.solve_ocp_postproc(result)
        t3 = time.time()

        if self.show_comp_stats:
            rospy.loginfo(
                'MP  - Comp stats: \n'
                '\n Prepr:  ' + str(round(1e3*((t1 - t0)), 1)) + ' ms' +
                '\n Proc:   ' + str(round(1e3*((t2 - t1)), 1)) + ' ms' +
                '\n Postpr: ' + str(round(1e3*((t3 - t2)), 1)) + ' ms' +
                '\n Optimal solution found: ' + str(result.success))

        # Publish the result to controller
        self._mp_result_topic.publish(result)
        self._mp_calc_time_topic.publish((t2 - t1))

    def solve_ocp_preproc(self, trigger):
        '''Shape the input arguments for the MPC update to the required format.

        :param trigger: trigger from controller
        :type trigger: truck_trailer_amr.Trigger_mp

        :return: solve_ocp_args - input argument for the solver
        :rtype: dict
        '''
        # Initial and final state and control, including mid-state
        x_t0 = np.array(trigger.x_t0)
        u_t0 = c.vertcat(trigger.u_t0)
        x_tm = np.array(trigger.x_tm)
        x_tf = np.array(trigger.x_tf)
        u_tf = c.vertcat(trigger.u_tf)

        # Prevent spinning multiple rounds
        x_tf[2] = (((x_tf[2] - x_t0[2]) - c.pi) % (2*c.pi)) - c.pi + x_t0[2]
        x_tf[3] = (((x_tf[3] - x_tf[2]) - c.pi) % (2*c.pi)) - c.pi + x_tf[2]

        # Define sequence of corridors for truck and trailer
        # Stage 1 - both in corridor 1
        # Stage 2 - truck in corridor 2, trailer in corridor 1 (or reversed)
        # Stage 3 - both in corridor 2
        W0 = c.vertcat(trigger.w0_stage1,
                       trigger.w0_stage2,
                       trigger.w0_stage3).reshape((3, 12))
        W1 = c.vertcat(trigger.w1_stage1,
                       trigger.w1_stage2,
                       trigger.w1_stage3).reshape((3, 12))

        # Close corridor depending on final corridor equality for both truck
        # and trailer
        closed_corridor_2 = 0
        closed_corridor_3 = 0
        if (trigger.w0_stage2 == trigger.w0_stage3) and (
                                    trigger.w1_stage2 == trigger.w1_stage3):
            closed_corridor_3 = 1
            if (trigger.w0_stage1 == trigger.w0_stage2) and (
                                    trigger.w1_stage1 == trigger.w1_stage2):
                closed_corridor_2 = 1

        # Get driving direction in corridor
        direction = trigger.direction

        # Combine all arguments
        solve_ocp_args = {'init_state': x_t0,
                          'init_control': u_t0,
                          'mid_state': x_tm,
                          'terminal_state': x_tf,
                          'terminal_control': u_tf,
                          'truck_corridor_vectors': W0,
                          'trailer_corridor_vectors': W1,
                          'closed_corridor_2': closed_corridor_2,
                          'closed_corridor_3': closed_corridor_3,
                          'direction': direction}

        # Debug failure to find solution
        # ------------------------------
        # print("Solution conditions were")
        # print('x_t0 =', x_t0)
        # print('u_t0 =', u_t0)
        # print('x_tm =', x_tm)
        # print('x_tf =', x_tf)
        # print('u_tf =', u_tf)
        # print('W0 = np.array(', W0, ')')
        # print('W1 = np.array(', W1, ')')
        # print('closed_corridor_2 = ', closed_corridor_2)
        # print('closed_corridor_3 = ', closed_corridor_3)
        # print('direction = ', direction)

        return solve_ocp_args

    def MPC_update(self, solve_ocp_args):
        '''MPC update computation. Publishes trajectories.

        :param solve_ocp_args: as defined in solve_ocp_preproc
        :type solve_ocp_args: dict

        :return: result - optimal solution
        :rtype: dict
        '''
        calc_succeeded = False

        # Catch error if no solution is found
        try:
            if not self.standalone:
                # Reset initialization if corridors are changing
                if not (self.old_corr ==
                        solve_ocp_args["truck_corridor_vectors"]).all():
                    rospy.loginfo("MP   - Resetting initialization")
                    self.opti_X = self.construct_init_guess(solve_ocp_args)

                # Solve optimal control problem
                [tsol_coarse, xsol_coarse, usol_coarse,
                 dT1sol, dT2sol, dT3sol, gist, self.opti_X] = self.solve_ocp(
                    solve_ocp_args["init_state"],
                    solve_ocp_args["init_control"],
                    solve_ocp_args["terminal_state"],
                    solve_ocp_args["terminal_control"],
                    solve_ocp_args["truck_corridor_vectors"],
                    solve_ocp_args["trailer_corridor_vectors"],
                    solve_ocp_args["closed_corridor_2"],
                    solve_ocp_args["closed_corridor_3"],
                    self.opti_X)

                self._mp_stage_time_topic.publish(StageTiming(
                    T1=dT1sol, T2=dT2sol, T3=dT3sol))
                self.old_corr = solve_ocp_args["truck_corridor_vectors"].full()

            else:
                stage_1, stage_2, stage_3 = self.stages
                stage_1.set_value(
                    self.x_t0, solve_ocp_args["init_state"])
                stage_1.set_value(
                    self.u_t0, solve_ocp_args["init_control"])
                stage_3.set_value(
                    self.x_tf, solve_ocp_args["terminal_state"])
                stage_3.set_value(
                    self.u_tf, solve_ocp_args["terminal_control"])
                w0_1 = solve_ocp_args["truck_corridor_vectors"][:, 0:4]
                w0_2 = solve_ocp_args["truck_corridor_vectors"][:, 4:8]
                w0_3 = solve_ocp_args["truck_corridor_vectors"][:, 8:12]
                w1_1 = solve_ocp_args["trailer_corridor_vectors"][:, 0:4]
                w1_2 = solve_ocp_args["trailer_corridor_vectors"][:, 4:8]
                w1_3 = solve_ocp_args["trailer_corridor_vectors"][:, 8:12]
                stage_1.set_value(self.w0_1, w0_1)
                stage_2.set_value(self.w0_2, w0_2)
                stage_3.set_value(self.w0_3, w0_3)
                stage_1.set_value(self.w1_1, w1_1)
                stage_2.set_value(self.w1_2, w1_2)
                stage_3.set_value(self.w1_3, w1_3)
                stage_2.set_value(self.closed_corridor_2,
                                  solve_ocp_args["closed_corridor_2"])
                stage_3.set_value(self.closed_corridor_3,
                                  solve_ocp_args["closed_corridor_3"])

                standalone_sol = self.ocp.solve()

                [tsol_coarse, xsol_coarse, usol_coarse,
                 dT1sol, dT2sol, dT3sol, gist] = \
                    self.get_standalone_solution(standalone_sol)

            calc_succeeded = True

        except Exception as e:
            if not self.standalone:
                rospy.logwarn(
                    'MP   - No solution found in MPC step. Error message:\n' +
                    str(e) +
                    '\nSolution conditions were' +
                    '\nx_t0 = ' + str(solve_ocp_args["init_state"]) +
                    '\nu_t0 = ' + str(solve_ocp_args["init_control"]) +
                    '\nx_tf = ' + str(solve_ocp_args["terminal_state"]) +
                    '\nu_tf = ' + str(solve_ocp_args["terminal_control"]) +
                    '\nW0 = np.array(' + str(
                        solve_ocp_args["truck_corridor_vectors"]) + ")" +
                    '\nW1 = np.array(' + str(
                        solve_ocp_args["trailer_corridor_vectors"]) + ")" +
                    '\nclosed_corridor_2 = ' + str(
                        solve_ocp_args["closed_corridor_2"]) +
                    '\nclosed_corridor_3 = ' + str(
                        solve_ocp_args["closed_corridor_3"]))
                tsol_coarse = c.DM.zeros(1)
                xsol_coarse = c.DM.zeros(4)
                usol_coarse = c.DM.zeros(2)
                dT1sol = 0.
                dT2sol = 0.
                dT3sol = 0.
                gist = 0.

            else:
                self.ocp.show_infeasibilities(1e-4)
                standalone_sol = self.ocp.non_converged_solution
                [tsol_coarse, xsol_coarse, usol_coarse,
                 dT1sol, dT2sol, dT3sol, gist] = \
                    self.get_standalone_solution(standalone_sol)

            calc_succeeded = False

        if self.show_comp_stats:
            rospy.loginfo("MP  - Solution of Time-to-goal: " +
                          str(dT1sol + dT2sol + dT3sol) + " s")

        result = {"tsol": tsol_coarse,
                  "xsol": xsol_coarse,
                  "usol": usol_coarse,
                  "Tsol": [dT1sol, dT2sol, dT3sol],
                  "gist": gist,
                  "success": calc_succeeded}

        return result

    def initialize_init_guess(self, ocp, stage_1, stage_2, stage_3,
                              x_1, x_2, x_3, u_1, u_2, u_3):
        '''Traces location of all optimization variables to be initialized in
        full optimization vector, which is given by opti.x. Procedure to be
        generalized in Rockit.

        :param ocp: optimal control problem
        :type ocp: rockit.Ocp

        :param stage_i: all stages
        :type stage_i: rockit.Stage

        :param x_i: states to be initialized
        :type x_i: casadi.MX

        :param u_i: controls to be initialized
        :type u_i: casadi.MX
        '''
        # Placeholders for the initial guesses for stage times
        dT1_s = ocp.value(stage_1.T)
        dT2_s = ocp.value(stage_2.T)
        dT3_s = ocp.value(stage_3.T)
        dT_s = c.horzcat(dT1_s, dT2_s, dT3_s)

        # Placeholders for initial guesses for states
        x_1_s0 = stage_1.sample(x_1[0], grid='control')[1]
        x_1_s1 = stage_1.sample(x_1[1], grid='control')[1]
        x_1_s2 = stage_1.sample(x_1[2], grid='control')[1]
        x_1_s3 = stage_1.sample(x_1[3], grid='control')[1]
        x_1_s = c.horzcat(x_1_s0, x_1_s1, x_1_s2, x_1_s3)
        x_2_s0 = stage_2.sample(x_2[0], grid='control')[1]
        x_2_s1 = stage_2.sample(x_2[1], grid='control')[1]
        x_2_s2 = stage_2.sample(x_2[2], grid='control')[1]
        x_2_s3 = stage_2.sample(x_2[3], grid='control')[1]
        x_2_s = c.horzcat(x_2_s0, x_2_s1, x_2_s2, x_2_s3)
        x_3_s0 = stage_3.sample(x_3[0], grid='control')[1]
        x_3_s1 = stage_3.sample(x_3[1], grid='control')[1]
        x_3_s2 = stage_3.sample(x_3[2], grid='control')[1]
        x_3_s3 = stage_3.sample(x_3[3], grid='control')[1]
        x_3_s = c.horzcat(x_3_s0, x_3_s1, x_3_s2, x_3_s3)
        x_s = c.horzcat(x_1_s, x_2_s, x_3_s)

        # Placeholders for initial guesses for controls
        u_1_s0 = stage_1.sample(u_1[0], grid='control')[1]
        u_1_s1 = stage_1.sample(u_1[1], grid='control')[1]
        u_1_s = c.horzcat(u_1_s0, u_1_s1)
        u_2_s0 = stage_2.sample(u_2[0], grid='control')[1]
        u_2_s1 = stage_2.sample(u_2[1], grid='control')[1]
        u_2_s = c.horzcat(u_2_s0, u_2_s1)
        u_3_s0 = stage_3.sample(u_3[0], grid='control')[1]
        u_3_s1 = stage_3.sample(u_3[1], grid='control')[1]
        u_3_s = c.horzcat(u_3_s0, u_3_s1)
        u_s = c.horzcat(u_1_s, u_2_s, u_3_s)

        # Placeholder for combined initial guesses
        TXU_s = c.horzcat(dT_s, x_s, u_s)

        # Trace location of all initial guesses in full optimization vector,
        # which is given by opti.x (procedure to be generalized in Rockit)
        TXU_J = c.jacobian(TXU_s, ocp._method.opti.x)
        GetJacobian = c.Function('GetJacobian', [], [TXU_J], [], ['Jac'])
        Jac = GetJacobian()['Jac'].full()
        self.Jac_inv = np.linalg.pinv(Jac)

    def construct_init_guess(self, solve_ocp_args):
        '''Constructs a vector which contains all optimization variables to be
        initialized, including time, states en controls. Initialization is done
        using linear interpolation between initial en terminal state.

        :param solve_ocp_args: as defined in solve_ocp_preproc
        :type solve_ocp_args: dict

        :return: opti_init - initial guess
        :rtype: casadi.DM
        '''
        x_t0 = solve_ocp_args["init_state"]
        x_tm = solve_ocp_args["mid_state"]
        x_tf = solve_ocp_args["terminal_state"]
        cl_co_2 = solve_ocp_args["closed_corridor_2"]
        cl_co_3 = solve_ocp_args["closed_corridor_3"]
        di = solve_ocp_args["direction"]
        N1 = self.opt['N_1']
        N2 = self.opt['N_2']
        N3 = self.opt['N_3']

        # Initialize time horizons depending on closed corridors
        dT1_init = 10.
        dT2_init = 1.
        dT3_init = 10.
        if cl_co_2 == 1:
            dT2_init = 0.04
        if cl_co_3 == 1:
            dT3_init = 0.04
        dT_init = c.horzcat([dT1_init, dT2_init, dT3_init])

        # Initial guesses for states.
        x_1_init0 = np.linspace(x_t0[0], x_tm[0], N1 + 1)
        x_1_init1 = np.linspace(x_t0[1], x_tm[1], N1 + 1)
        x_1_init2 = np.linspace(x_t0[2], x_tm[2], N1 + 1)
        x_1_init3 = np.linspace(x_t0[3], x_tm[3], N1 + 1)
        x_1_init = c.vertcat(x_1_init0, x_1_init1,
                             x_1_init2, x_1_init3)
        x_2_init0 = np.linspace(x_tm[0], x_tm[0], N2 + 1)
        x_2_init1 = np.linspace(x_tm[1], x_tm[1], N2 + 1)
        x_2_init2 = np.linspace(x_tm[2], x_tm[2], N2 + 1)
        x_2_init3 = np.linspace(x_tm[3], x_tm[3], N2 + 1)
        x_2_init = c.vertcat(x_2_init0, x_2_init1,
                             x_2_init2, x_2_init3)
        x_3_init0 = np.linspace(x_tm[0], x_tf[0], N3 + 1)
        x_3_init1 = np.linspace(x_tm[1], x_tf[1], N3 + 1)
        x_3_init2 = np.linspace(x_tm[2], x_tf[2], N3 + 1)
        x_3_init3 = np.linspace(x_tm[3], x_tf[3], N3 + 1)
        x_3_init = c.vertcat(x_3_init0, x_3_init1,
                             x_3_init2, x_3_init3)
        x_init = c.vertcat(x_1_init, x_2_init, x_3_init)

        # Initial guesses for controls.
        v_1 = np.sqrt((x_tm[0] - x_t0[0])**2 +
                      (x_tm[1] - x_t0[1])**2)/dT1_init
        v_2 = np.sqrt((x_tm[0] - x_tm[0])**2 +
                      (x_tm[1] - x_tm[1])**2)/dT2_init
        v_3 = np.sqrt((x_tf[0] - x_tm[0])**2 +
                      (x_tf[1] - x_tm[1])**2)/dT3_init
        # Negative velocity if vehicle should be driving backwards
        if di[0] == 1:
            v_1 = -v_1
        if di[1] == 1:
            v_3 = -v_3

        u_1_init0 = np.linspace(v_1, v_1, N1 + 1)
        u_1_init1 = np.linspace(0., 0., N1 + 1)
        u_1_init = c.vertcat(u_1_init0, u_1_init1)
        u_2_init0 = np.linspace(v_2, v_2, N2 + 1)
        u_2_init1 = np.linspace(0., 0., N2 + 1)
        u_2_init = c.vertcat(u_2_init0, u_2_init1)
        u_3_init0 = np.linspace(v_3, v_3, N3 + 1)
        u_3_init1 = np.linspace(0., 0., N3 + 1)
        u_3_init = c.vertcat(u_3_init0, u_3_init1)
        u_init = c.vertcat(u_1_init, u_2_init, u_3_init)

        # Combine time, state and controls and calculate initial guess vector
        TXU_init = c.vertcat(dT_init, x_init, u_init)
        opti_init = c.vertcat(self.Jac_inv.dot(TXU_init))

        return opti_init

    def get_standalone_solution(self, standalone_sol):
        '''Get solution for standalone calculation.

        :param standalone_sol: standalone solution
        :type standalone_sol: rockit.solution.OcpSolution

        :return: (tsol_coarse, xsol_coarse, usol_coarse, dTisol, gist)

            - tsol_coarse (casadi.DM) - time on control grid
            - xsol_coarse (casadi.DM) - states on control grid
            - usol_coarse (casadi.DM) - controls on control grid
            - dTisol (casadi.DM) - time duration of each stage
            - gist (casadi.DM) - all numerical information needed to compute\
                                 any value/sample
        '''
        tsol_coarse = []
        xsol_coarse = []
        usol_coarse = []
        dTsol = []

        # Get time, state and control from every stage
        for stage, x, u in zip(self.stages, self.x123, self.u123):
            t_sol = standalone_sol(stage).sample(x, grid='control')[0]
            x_sol = standalone_sol(stage).sample(x, grid='control')[1]
            u_sol = standalone_sol(stage).sample(u, grid='control')[1]
            dT_sol = standalone_sol(stage).value(stage.T)
            tsol_coarse.append(t_sol)
            xsol_coarse.append(x_sol)
            usol_coarse.append(u_sol)
            dTsol.append(dT_sol)

        dT1sol, dT2sol, dT3sol = dTsol
        gist = standalone_sol.gist
        tsol_coarse = c.vertcat(*tsol_coarse)
        xsol_coarse = c.vertcat(*xsol_coarse).T
        usol_coarse = c.vertcat(*usol_coarse).T

        return (tsol_coarse, xsol_coarse, usol_coarse,
                dT1sol, dT2sol, dT3sol, gist)

    def solve_ocp_postproc(self, sol):
        '''Shape the solution to the required format.

        :param sol: raw solution
        :type sol: dict

        :return: result - postprocessed solution, all time, state and control
                 information on the fine and coarse grid.
        :rtype: truck_trailer_amr.msg.Trajectories
        '''
        # Give zeros if not successful
        if not sol["success"]:
            xsol_fine = np.zeros((4, 100))
            usol_fine = np.zeros((2, 100))
            udotsol_fine = np.zeros((2, 100))
            t_fine = np.zeros((1, 100))
            t_coarse = np.zeros((1, 100))
            stage_indices = [0, 0]
        else:
            xsol_fine = []
            usol_fine = []
            udotsol_fine = []
            t_fine = []
            t_coarse = []
            stage_indices = []

            dT1, dT2, dT3 = sol["Tsol"]
            N_1 = self.opt['N_1']
            N_2 = self.opt['N_2']
            N_3 = self.opt['N_3']

            Ts = self.opt["Ts"]
            mp_update_index = self.opt["mp_update_index"]
            sampler1, sampler2, sampler3 = self.samplers
            gist = sol["gist"]

            # Build solution vector with stage1 taking stage time into account
            t1 = np.arange(0., min(dT1, Ts*mp_update_index), Ts)
            t1_coarse = sol["tsol"][0:N_1+1]
            t_coarse.append(t1_coarse)
            # Check if stage time is too small
            if len(t1) < 1:
                t1_f = 0.
            else:
                [x1sol, u1sol, udot1sol] = sampler1(gist, c.DM(t1).T)
                t_fine.append(t1.T)
                xsol_fine.append(x1sol.T)
                usol_fine.append(u1sol.T)
                udotsol_fine.append(udot1sol.T)
                t1_f = t1[-1]

            # Take (parts from) stage2 if stage1 was too short
            if dT1 <= Ts*mp_update_index:
                t2 = np.arange(Ts, min(dT2, Ts*mp_update_index - t1_f), Ts)
            else:
                t2 = np.array([])
            t2_coarse = sol["tsol"][N_1+1:N_1+N_2+2] + t1_coarse[-1]
            t_coarse.append(t2_coarse)
            # Check if stage time is too small
            if len(t2) < 1:
                t2_f = t1_f
            else:
                [x2sol, u2sol, udot2sol] = sampler2(gist, c.DM(t2).T)
                t_fine.append(t2.T + t1_f)
                xsol_fine.append(x2sol.T)
                usol_fine.append(u2sol.T)
                udotsol_fine.append(udot2sol.T)
                t2_f = t2[-1] + t1_f

            # Take (parts from) stage3 if stage1 + stage2 was too short
            if dT1 + dT2 <= Ts*mp_update_index:
                t3 = np.arange(Ts, min(dT3, Ts*mp_update_index - t2_f), Ts)
            else:
                t3 = np.array([])
            t3_coarse = sol["tsol"][N_1+N_2+2:N_1+N_2+N_3+3] + t2_coarse[-1]
            t_coarse.append(t3_coarse)
            # Check if stage time is too small
            if len(t3) > 1:
                [x3sol, u3sol, udot3sol] = sampler3(gist, c.DM(t3).T)
                t_fine.append(t3.T + t2_f)
                xsol_fine.append(x3sol.T)
                usol_fine.append(u3sol.T)
                udotsol_fine.append(udot3sol.T)

            # Concatenate all results
            t_fine = np.concatenate(t_fine)
            xsol_fine = np.concatenate(xsol_fine)
            usol_fine = np.concatenate(usol_fine)
            udotsol_fine = np.concatenate(udotsol_fine)
            t_coarse = np.concatenate(t_coarse)

            # Get cumulative stage indices
            ti = [len(t1), len(t2), len(t3)]
            stage_indices = np.cumsum(ti)

        xsol_coarse = sol["xsol"]
        usol_coarse = sol["usol"]

        # Convert solution to standard type (lists).
        [xsol_coarse, xsol_fine, usol_coarse, usol_fine] = \
            helpers.solution_to_lists(
                [xsol_coarse, xsol_fine.T, usol_coarse, usol_fine.T])

        result = Trajectories(
            t=t_fine,
            t_coarse=t_coarse,

            px1=xsol_fine[0],
            py1=xsol_fine[1],
            theta1=xsol_fine[2],
            theta0=xsol_fine[3],
            px1_coarse=xsol_coarse[0],
            py1_coarse=xsol_coarse[1],
            theta1_coarse=xsol_coarse[2],
            theta0_coarse=xsol_coarse[3],

            v0=usol_fine[0],
            omega0=usol_fine[1],
            v0_coarse=usol_coarse[0],
            omega0_coarse=usol_coarse[1],

            stage_indices=stage_indices,
            success=sol["success"])
        return result


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
