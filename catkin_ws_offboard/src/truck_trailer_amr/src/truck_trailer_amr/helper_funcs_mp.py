#!/usr/bin/env python3
import numpy as np
import casadi as c
from multipledispatch import dispatch


#########################
# Motionplanner helpers #
#########################


def solution_to_lists(sol):
    '''Convert the CasADi objects that are the solution of the optimization
    to lists in order to return them to Controller.

    :param sol: solution, list with CasADi arrays
    :type sol: casadi.DM

    :return: sol - cast solution
    :rtype: lst[]
    '''
    for i in range(len(sol)):
        sol[i] = np.array(sol[i]).tolist()

    return sol


@dispatch(float, float, float, float, float, float, float)
def get_vehicle_vertices(x, y, theta, w_left, w_right, l_front, l_back):
    '''Get all vertices of a vehicle, using following convention::


        (4)       y     (3)   ^
        .|--------|------|    |  w_left
        .|        o---x  |    x
        .|               |    |
        .|---------------|    |  w_right
        (1)             (2)   v
        . <-------><---->
        .  l_back   l_front


    :param x, y: center point of vehicle
    :type x, y: float

    :param theta: angle wrt x-axis
    :type theta: float

    :param w_left: width of vehicle, to the left of center
    :type w_left: float

    :param w_right: width of vehicle, to the right of center
    :type w_right: float

    :param l_front: length of vehicle, front to center
    :type l_front: float

    :param l_back: length of vehicle, back to center
    :type l_back: float

    :return: points - matrix with all vertices
    :rtype: np.ndarray
    '''
    # Vertices of vehicle in homogeneous coordinates
    vertices = np.array([[-l_back, l_front, l_front, -l_back],
                         [-w_right, -w_right, w_left, w_left],
                         [0, 0, 0, 0],
                         [1, 1, 1, 1]])

    # Homogeneous transformation matrix
    homog_transf_matrix = np.array([[c.cos(theta), -c.sin(theta), 0, x],
                                    [c.sin(theta),  c.cos(theta), 0, y],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

    # Transform vertices and extract 2D points
    points = homog_transf_matrix.dot(vertices)[:2].transpose()

    return points


@dispatch(c.MX, c.MX, c.MX, float, float, float, float)
def get_vehicle_vertices(x, y, theta, w_left, w_right, l_front, l_back):
    '''Get all vertices of a vehicle, using following convention::


        (4)       y     (3)   ^
        .|--------|------|    |  w_left
        .|        o---x  |    x
        .|               |    |
        .|---------------|    |  w_right
        (1)             (2)   v
        . <-------><---->
        .  l_back   l_front


    :param x, y: center point of vehicle
    :type x, y: c.MX

    :param theta: angle wrt x-axis
    :type theta: c.MX

    :param w_left: width of vehicle, to the left of center
    :type w_left: float

    :param w_right: width of vehicle, to the right of center
    :type w_right: float

    :param l_front: length of vehicle, front to center
    :type l_front: float

    :param l_back: length of vehicle, back to center
    :type l_back: float

    :return: points - matrix with all vertices
    :rtype: np.ndarray
    '''
    # Vertices of vehicle in 3D homogeneous coordinates
    vertices = c.vertcat(
        c.horzcat(-l_back, l_front, l_front, -l_back),
        c.horzcat(-w_right, -w_right, w_left, w_left),
        c.horzcat(0, 0, 0, 0),
        c.horzcat(1, 1, 1, 1))

    # Homogeneous transformation matrix
    homog_transf_matrix = c.vertcat(
        c.horzcat(c.cos(theta), -c.sin(theta), 0, x),
        c.horzcat(c.sin(theta),  c.cos(theta), 0, y),
        c.horzcat(0, 0, 1, 0),
        c.horzcat(0, 0, 0, 1))

    # import pdb; pdb.set_trace()
    # Transform vertices and extract 2D points
    points = (homog_transf_matrix @ vertices)[0:2, :]

    return points
