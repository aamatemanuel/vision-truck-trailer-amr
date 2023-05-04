#!/usr/bin/env python3
from geometry_msgs.msg import PointStamped

import rospy
import ruamel.yaml
import io
import rospkg
import numpy as np


class CorridorGenerator:
    '''A class to generate corridors and write them to a .yaml file by clicking
    on an Rviz map.
    '''

    def __init__(self):
        '''Constructor for CorridorGenerator class.
        '''
        rospy.init_node('corridor_generator')
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_cb)

        # Init data
        self.routes = dict()
        self.corridors = dict()
        self.corridor_points = []
        self.route_i = 0

    def clicked_point_cb(self, point):
        '''Add the clicked point to the current list of points. When 6 points
        have been clicked, generate a corridor + waypoint from them and clear
        the list of points to start a new corridor.

        4 points are the corners, 5th point is the waypoint location, 6th point
        points in the direction relative to 5th point of the desired
        orientation at the waypoint.

        Always click the corners in clockwise order!!

        Arg:
            point: geometry_msgs/PointStamped
        '''
        rospy.loginfo("Clicked point " + str(point))
        # Add incoming point to corridor_points.
        np_point2d = np.array([point.point.x, point.point.y])
        self.corridor_points.append(np_point2d)

        # Process corridor
        if len(self.corridor_points) == 6:
            corridor = self.get_corridor_def(self.corridor_points)
            self.corridors.update(
                {"corridor"+str(len(self.corridors)): corridor})
            self.routes["route"+str(self.route_i)] = self.corridors
            self.corridor_points = []
            rospy.loginfo('Added corridor!')

            self.write_yaml(self.routes)

    def get_corridor_def(self, points):
        '''From the list of points, get the normal vectors and format corridor.
        From the two possible normal vectors of a line segment we select,
        looking from point1 to point2, the one on the left (hence, selecting
        points clockwise).

        Arg:
            points: [np.array([px, py]), np.array([px, py])]
        '''
        corridor = dict()
        # Get the normal vectors from the corners = first 4 points
        # 0-1, 1-2, 2-3:
        for i in range(3):
            p0 = points[i]
            p1 = points[i+1]

            n = np.array([-(p1[1]-p0[1]), p1[0]-p0[0]])
            n = n/np.linalg.norm(n)
            t = points[i]
            corridor.update({'n' + str(i+1): n.tolist()})
            corridor.update({'t' + str(i+1): t.tolist()})
        # 3-0:
        p0 = points[3]
        p1 = points[0]

        n = np.array([-(p1[1]-p0[1]), p1[0]-p0[0]])
        n = n/np.linalg.norm(n)
        t = points[3]
        corridor.update({'n' + str(4): n.tolist()})
        corridor.update({'t' + str(4): t.tolist()})

        # Waypoint:
        dp = (points[5] - points[4])
        theta = np.arctan2(dp[1], dp[0]).item()  # .item() to np.float -> float
        corridor.update({'waypoint': [*points[4].tolist(), theta, theta]})

        print(corridor)

        return corridor

    def write_yaml(self, routes):
        '''Write the current corridors to a yaml file.

        Arg:
            corridors: [corridor, corridor, ...]
        '''

        rospack = rospkg.RosPack()
        pck_path = rospack.get_path('rviz_floor_map')
        yaml = ruamel.yaml.YAML()
        # route = {'routes': routes}
        with io.open(pck_path + '/corridors/corridors.yaml', 'w') as outfile:
            yaml.dump(routes, outfile)

        rospy.loginfo("Corridors written to file. Start with the next " +
                      "corridor or shut down node with Ctrl-C")
        new_route = int(input("To continue this route press 0." +
                              "\nTo start new subroute press 1.\n"))

        if new_route not in {0, 1}:
            rospy.logerr("You selected " + str(new_route) + ", which is not" +
                         "0 or 1.")
        elif new_route == 1:
            rospy.loginfo("Starting new subroute. Start clicking " +
                          "on the map.")
            self.corridors = dict()
            self.route_i += 1


if __name__ == '__main__':
    corridor_generator = CorridorGenerator()
    # corridors = {
    #     'corridor0':
    #         {'n1': [0.4074243903160095, 2.1592406034469604],
    #          't1': [0.4390941858291626, 0.888481616973877],
    #          'n2': [1.7315308451652527, 0.24443840980529785],
    #          't2': [2.598334789276123, 0.48105722665786743],
    #          'n3': [0.24444615840911865, -2.0573915243148804],
    #          't3': [2.842773199081421, -1.2504736185073853],
    #          'n4': [-1.0898427069187164, 1.1611042022705078],
    #          't4': [0.7853816747665405, -1.494919776916504],
    #          'waypoint':
    #             [1.2833229303359985, -0.2138982117176056,
    #              -0.16515706019781345, -0.16515706019781345]}}
    # corridor_generator.write_yaml(corridors)

    rospy.loginfo("Ready. Start clicking on the map.")
    rospy.spin()
