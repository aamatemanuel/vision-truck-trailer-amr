#!/usr/bin/env python3
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from truck_trailer_amr.rviz_handler import RvizHandler
import truck_trailer_amr.helper_funcs_mp as helpers_mp
import truck_trailer_amr.helper_funcs_ctrl as helpers_ctrl
import numpy as np
import rospy
from casadi import vertcat


class Corridor:
    '''Class to characterize corridors.
    A corridor is defined by the plane parameter vectors of four planes/lines
    delimiting the area of the corridor. The vectors are assembled in a Numpy
    array such that they can easily be cast to lists for sending them over
    ROS topics.
    '''
    def __init__(self, n, t, target, direction):
        '''Constructor for the Corridor class.

        :param n: normal vector for each of the planes, stacked together in one
                  array
                  e.g. np.concatenate((np.array([1, 0]), np.array([0, 1])))
        :type n: np.array
        :param t: point on the plane for each of the planes, similarly stacked
               together in an np.array([])
        :type t: np.array

        :param target: destination point in this corridor
        :type target: np.array
        '''
        assert not len(n) % 2, "Array length of n is not a multiple of 2"
        assert len(t) == len(n), "Arrays n and t are of different size"

        # Construct edge parameter vectors
        W = []

        for i in range(int(len(n)/2)):
            j = 2*i

            nj = n[j:j+2]
            tj = t[j:j+2]
            W.append(nj)
            W.append(np.array([-nj.T @ tj]))
        self.W = np.concatenate(W)

        # Construct corners
        self.corners = []

        # First corner double to make the visualization a full polygon.
        W = [*self.W, *self.W[0:3]]
        for i in range(0, len(self.W), 3):
            w1 = W[i:i+3]
            w2 = W[i+3:i+6]
            self.corners.append(Point(*self.get_intersection(w1, w2), 0))

        # Store target
        self.target = target
        self.direction = direction

    def get_intersection(self, w1, w2):
        '''Use Cramer's rule to solve the system of equations
        ``w1.T @ ph = 0 ; w2.T @ ph = 0;`` for p, where ph = [p; 1]

        :param np.array w1: line parameter vector of shape [w0, w1, w2]
        :param np.array w2: line parameter vector of shape [w0, w1, w2]

        :return: intersection of w1 and w2
        :rtype: np.array
        '''
        return np.array(
            [(w1[1]*w2[2] - w1[2]*w2[1])/(w1[0]*w2[1] - w1[1]*w2[0]),
             (w1[2]*w2[0] - w1[0]*w2[2])/(w1[0]*w2[1] - w1[1]*w2[0])])


class CorridorHandler:
    '''Class to get plane parameter vectors for the stages in multistage ocp,
    given the current location in the environment.
    '''
    def __init__(self):
        '''Constructor for CorridorHandler class.
        '''
        mp_params = rospy.get_param('/motionplanner')
        self.veh = mp_params['vehicles']

        self.current_route = []
        self.reset()

    def reset(self):
        '''Reset route.
        '''
        self.corridor_iter = [0, 0]  # [Truck, Trailer]
        self.route_iter = 0
        self.rviz_handler = RvizHandler()
        self.route_params = rospy.get_param("/routes")
        try:
            self.visualize_all_routes()
            route = self.load_route(self.route_params["route0"])
            self.prepare_current_route(route)
            self.rviz_handler.draw_active_corridors(self.current_route[:2])
        except Exception as e:
            rospy.logerr('CORR - No routes given. %s' % e)

        self.trigger_corridor_change = rospy.Publisher(
                '/controller/corridor_change', Bool, queue_size=1)

    def visualize_all_routes(self):
        '''Visualize all routes.
        '''
        routes = self.route_params
        routes_visual = []
        for route_iter in range(len(routes)):
            route_name = "route" + str(route_iter)
            routes_visual.extend(self.load_route(routes[route_name]))
        self.rviz_handler.draw_all_routes(routes_visual)

    def load_route(self, route):
        '''Load a single route (consisting of multiple corridors).

        :param dict route: sequence of corridors with n, t and waypoint, named
                      as corridor0, corridor1,...

        :return: route - list of Corridor instances
        :rtype: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        corridors = {}
        for corridor_iter in range(len(route)):
            corridor_name = "corridor" + str(corridor_iter)
            corridor = route[corridor_name]
            n = np.concatenate((corridor['n1'], corridor['n2'],
                                corridor['n3'], corridor['n4']))
            t = np.concatenate((corridor['t1'], corridor['t2'],
                                corridor['t3'], corridor['t4']))
            target = np.array(corridor['waypoint'])
            direction = corridor['direction']
            corridors[corridor_name] = Corridor(n, t, target, direction)
        route = [corridors["corridor" + str(i)] for i in range(len(route))]
        return route

    def load_next_route(self):
        '''Load next route if target reached.
        '''
        self.route_iter += 1
        self.corridor_iter = [0, 0]
        try:
            route_name = "route" + str(self.route_iter)
            route = self.load_route(self.route_params[route_name])
            self.prepare_current_route(route)
        except Exception as e:
            rospy.loginfo('CORR - Finished last subroute.')
            self.reset()

    def prepare_current_route(self, route):
        '''Prepare the current route of corridors by deciding on the travel
        direction and recalculate target point of each corridor to improve
        initialization process later on.

        :param route: sequence of Corridor instances
        :type route: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        self.current_route = route

        for i in range(len(route)-1):
            target_angle_0 = (route[i].target[2] + route[i].target[3])/2
            target_angle_1 = (route[i+1].target[2] + route[i+1].target[3])/2
            target_angle = (target_angle_0 + target_angle_1)/2
            self.current_route[i].target[2] = target_angle
            self.current_route[i].target[3] = target_angle

        self.mp_last_goal = self.current_route[-1].target

        # Check feasibility of target in corridor
        for i in range(len(route)):
            self.check_target_feasibility(self.current_route[i])
        self.rviz_handler.draw_active_route(self.current_route)
        self.rviz_handler.draw_corridor_targets(self.current_route)

    def get_mid_line(self, corridor):
        '''[DEPRECATED] Get coordinates of mid line though corridor

        :param corridor: corridor of which to get the mid line
        :type corridor: truck_trailer_amr.corridor_handler.Corridor
        '''
        dist = []
        target = Point(corridor.target[0], corridor.target[1], 0.)
        for j in range(len(corridor.corners)):
            dist.append(helpers_ctrl.position_diff_norm(
                target, corridor.corners[j]))

        dist_sorted = sorted(range(len(dist)), key=lambda k: dist[k])

        x_end, y_end, z_end = helpers_ctrl.get_mean(
            corridor.corners[dist_sorted[0]], corridor.corners[dist_sorted[1]])

        x_sta, y_sta, z_sta = helpers_ctrl.get_mean(
            corridor.corners[dist_sorted[2]], corridor.corners[dist_sorted[3]])
        mid_end = Point(x_end, y_end, z_end)
        mid_sta = Point(x_sta, y_sta, z_sta)
        return mid_sta, mid_end

    def check_target_feasibility(self, corridor):
        '''Check feasibility of target in corridor.

        :param corridor: Corridor to check feasibility in.
        :type corridor: truck_trailer_amr.corridor_handler.Corridor
        '''
        L0 = self.veh['truck']['L']
        M0 = self.veh['truck']['M']
        W0 = self.veh['truck']['W']
        L1 = self.veh['trailer1']['L']
        M1 = self.veh['trailer1']['M']
        W1 = self.veh['trailer1']['W']

        px1, py1, theta1, theta0 = corridor.target
        px0 = px1 + L1*np.cos(theta1) + M0*np.cos(theta0)
        py0 = py1 + L1*np.sin(theta1) + M0*np.sin(theta0)

        trailer_vertices = helpers_mp.get_vehicle_vertices(
                                    px1, py1, theta1, W1/2, W1/2, L1, M1)
        truck_vertices = helpers_mp.get_vehicle_vertices(
                                  px0, py0, theta0, W0/2, W0/2, L0, M0)

        self.check_vertex_feasibility(corridor, trailer_vertices)
        self.check_vertex_feasibility(corridor, truck_vertices)

        corridor.trailer_corners = []
        corridor.truck_corners = []
        for vertex in trailer_vertices:
            corridor.trailer_corners.append(Point(vertex[0], vertex[1], 0))
        for vertex in truck_vertices:
            corridor.truck_corners.append(Point(vertex[0], vertex[1], 0))

    def check_vertex_feasibility(self, corridor, vehicle_vertices):
        '''Check feasibility of vehicle vertices in corridor.

        :param corridor: Corridor
        :type corridor: truck_trailer_amr.corridor_handler.Corridor

        :param np.array[] vehicle_vertices: list of vehicle vertices

        :return: True if feasible
        :rtype: bool
        '''
        for vertex in vehicle_vertices:
            phom = vertcat(vertex[0], vertex[1], 1)
            vertex_in_corridor = corridor.W.reshape((4, 3)) @ phom
            for i in range(4):
                if vertex_in_corridor[i] > 0.:
                    return 0
        return 1

    def overlap(self):
        '''Returns true if truck and trailer are in different corridors.

        :return: overlap
        :rtype: bool
        '''
        return self.corridor_iter[0] != self.corridor_iter[1]

    def toggle_check(self, mp_traj, index_start, mp_index, mp_update_index,
                     mp_goal):
        '''Check whether it is possible to toggle the trailer or truck
        corridor. If yes, toggle the corridor.
        A toggle is possible if all the following points on the trajectory
        are feasible in the new corridor.

        :param mp_traj: trajectory
        :type mp_traj: dict

        :param index_start: index from which to start the toggle check
        :type index_start: int

        :param mp_index: index of the current trajectory point
        :type mp_index: int

        :param mp_update_index: update index
        :type mp_update_index:  int

        :return: mp_goal
        :rtype: np.array
        '''

        L0 = self.veh['truck']['L']
        M0 = self.veh['truck']['M']
        W0 = self.veh['truck']['W']
        L1 = self.veh['trailer1']['L']
        M1 = self.veh['trailer1']['M']
        W1 = self.veh['trailer1']['W']

        trailer_feas = []
        truck_feas = []
        px1 = mp_traj['px1'][index_start]
        py1 = mp_traj['py1'][index_start]
        theta1 = mp_traj['theta1'][index_start]
        theta0 = mp_traj['theta0'][index_start]
        px0 = px1 + L1*np.cos(theta1) + M0*np.cos(theta0)
        py0 = py1 + L1*np.sin(theta1) + M0*np.sin(theta0)

        trailer_vertices = helpers_mp.get_vehicle_vertices(
                            px1, py1, theta1, W1/2, W1/2, L1, M1)
        truck_vertices = helpers_mp.get_vehicle_vertices(
                            px0, py0, theta0, W0/2, W0/2, L0, M0)
        trailer = []
        truck = []
        for vertex in trailer_vertices:
            trailer.append(Point(vertex[0], vertex[1], 0))
        for vertex in truck_vertices:
            truck.append(Point(vertex[0], vertex[1], 0))
        self.rviz_handler.draw_vehicles_x0(trailer, truck)

        # Check if new trigger point is beyond corridor checkpoint,
        # meaning a new corridor should be toggled.
        if not self.last():
            desired_index_start = mp_index + mp_update_index
            T_fine_desired = mp_traj['t'][desired_index_start]
            coarse_index_start = next(i for i, T_coarse in enumerate(
                mp_traj['t_coarse']) if T_coarse >= T_fine_desired)
            index_end = len(mp_traj['px1_coarse'])
            # Take corridor index of last vehicle i.e.
            # driving backwards = truck, driving forward = trailer
            current_corridor_index = self.corridor_low()
            next_corridor_index = current_corridor_index + 1
            next_corridor = \
                self.current_route[next_corridor_index]
            trailer_feas = []
            truck_feas = []
            px1_lst = []
            py1_lst = []
            for i in range(coarse_index_start, index_end):
                px1 = mp_traj['px1_coarse'][i]
                py1 = mp_traj['py1_coarse'][i]
                theta1 = mp_traj['theta1_coarse'][i]
                theta0 = mp_traj['theta0_coarse'][i]
                px0 = px1 + L1*np.cos(theta1) + M0*np.cos(theta0)
                py0 = py1 + L1*np.sin(theta1) + M0*np.sin(theta0)

                px1_lst.append(px1)
                py1_lst.append(py1)

                trailer_vertices = helpers_mp.get_vehicle_vertices(
                                px1, py1, theta1, W1/2, W1/2, L1, M1)
                truck_vertices = helpers_mp.get_vehicle_vertices(
                                    px0, py0, theta0, W0/2, W0/2, L0, M0)
                trailer_feas.append(
                    self.check_vertex_feasibility(
                        next_corridor, trailer_vertices))
                truck_feas.append(
                    self.check_vertex_feasibility(
                        next_corridor, truck_vertices))

            if self.get_active_corridors_directions()[1]:
                # Entering backwards
                # When entering overlap region (corresponding to stage 2),
                # switch to overlap status. When leaving overlap region,
                # switch to next corridor.
                if self.overlap():
                    if sum(truck_feas) >= len(truck_feas):
                        rospy.loginfo(
                            'CTRL - Truck toggle - Backward - Overlap')
                        self.toggle('truck')
                        mp_goal = self.get_corridor_target()
                        self.trigger_corridor_change.publish(1)
                else:
                    if sum(trailer_feas) >= len(trailer_feas):
                        rospy.loginfo('CTRL - Trailer toggle - ' +
                                      'Backward - No overlap')
                        self.toggle('trailer')
                        mp_goal = self.get_corridor_target()
                        self.trigger_corridor_change.publish(1)
            else:
                # Entering forwards
                if self.overlap():
                    if sum(trailer_feas) >= len(trailer_feas):
                        rospy.loginfo(
                            'CTRL - Trailer toggle - Forward - Overlap')
                        self.toggle('trailer')
                        mp_goal = self.get_corridor_target()
                        self.trigger_corridor_change.publish(1)
                else:
                    if sum(truck_feas) >= len(truck_feas):
                        rospy.loginfo(
                            'CTRL - Truck toggle - Forward - No overlap')
                        self.toggle('truck')
                        mp_goal = self.get_corridor_target()
                        self.trigger_corridor_change.publish(1)

        return mp_goal

    def toggle(self, vehicle):
        '''Increase the corridor index for the given vehicle.

        :param vehicle: determines whether to increase truck or trailer
        :type vehicle: str
        '''
        if not self.last():
            if vehicle == 'truck':
                self.corridor_iter[0] += 1
            elif vehicle == 'trailer':
                self.corridor_iter[1] += 1
        route = self.current_route[self.corridor_low():self.corridor_low()+2]
        self.rviz_handler.draw_active_corridors(route)

    def last(self):
        '''Returns True if at final corridor.

        :return: at final corridor
        :rtype: bool
        '''
        return (self.corridor_iter[0] == self.corridor_iter[1] ==
                len(self.current_route)-1)

    def corridor_low(self):
        '''Returns lowest number of two active corridors.

        :return: lowest corridor iter
        :rtype: int
        '''
        return min(self.corridor_iter)

    def get_corridor_target(self):
        '''Returns target of current corridor for motion planner.

        :return: active_corridors_targets
        :rtype: np.ndarray
        '''
        return self.get_active_corridors_targets()[1]

    def get_active_corridors_targets(self):
        '''Returns targets for both active corridors.

        :return: [target0, target1]
        :rtype: np.ndarray[]
        '''
        target0 = self.current_route[self.corridor_low()].target
        if self.last():
            target1 = target0
        else:
            target1 = self.current_route[self.corridor_low()+1].target
        return [target0, target1]

    def get_active_corridors_directions(self):
        '''Returns directions for both active corridors.

        :return: directions - [1/0, 1/0]
        :rtype: np.array
        '''
        direction0 = self.current_route[self.corridor_low()].direction[0]
        if self.last():
            direction1 = direction0
        else:
            direction1 = self.current_route[self.corridor_low()+1].direction[0]
        return np.array([direction0, direction1])

    def get_active_corridors_vectors(self):
        '''Returns the w vectors to apply as constraints in motionplanner.

        :return: (w0_stage1, w0_stage2, w0_stage3, w1_stage1, w1_stage2,
                  w1_stage3)

            - wi_stagej (np.ndarray) - parameter vectors for vehicle i in
                                       stage j
        '''
        Wcorr0 = self.current_route[self.corridor_low()].W
        if self.last():
            Wcorr1 = Wcorr0
        else:
            Wcorr1 = self.current_route[self.corridor_low()+1].W

        if not self.get_active_corridors_directions()[1]:
            # Entering forwards
            if not self.overlap():
                w0_stage1 = Wcorr0
                w0_stage2 = Wcorr1
                w0_stage3 = Wcorr1

                w1_stage1 = Wcorr0
                w1_stage2 = Wcorr0
                w1_stage3 = Wcorr1
            else:
                w0_stage1 = Wcorr1
                w0_stage2 = Wcorr1
                w0_stage3 = Wcorr1

                w1_stage1 = Wcorr0
                w1_stage2 = Wcorr1
                w1_stage3 = Wcorr1
        else:
            # Entering backwards
            if not self.overlap():
                w0_stage1 = Wcorr0
                w0_stage2 = Wcorr0
                w0_stage3 = Wcorr1

                w1_stage1 = Wcorr0
                w1_stage2 = Wcorr1
                w1_stage3 = Wcorr1
            else:
                w0_stage1 = Wcorr0
                w0_stage2 = Wcorr1
                w0_stage3 = Wcorr1

                w1_stage1 = Wcorr1
                w1_stage2 = Wcorr1
                w1_stage3 = Wcorr1

        return w0_stage1, w0_stage2, w0_stage3, w1_stage1, w1_stage2, w1_stage3


if __name__ == "__main__":
    # Test
    n1 = np.array([1, 0])
    n2 = np.array([0, 1])
    n3 = np.array([-1, 0])
    n4 = np.array([0, -1])
    n = np.concatenate((n1, n2, n3, n4))

    t1 = np.array([1, 0.5])
    t2 = np.array([0.5, 1])
    t3 = np.array([0, 0.5])
    t4 = np.array([0.5, 0.])
    t = np.concatenate((t1, t2, t3, t4))

    target = np.array([0., 0.])

    corr = Corridor(n, t, target)
