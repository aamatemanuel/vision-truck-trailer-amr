#!/usr/bin/env python3

from geometry_msgs.msg import Point, PoseStamped, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import truck_trailer_amr.helper_funcs_ctrl as helpers
import rospy


class RvizHandler(object):
    '''A class to take care of Rviz markers for the controller.
    '''
    def __init__(self):
        self._marker_setup()

        # Params
        self.room = rospy.get_param('/motionplanner/environment/room_size')
        self.room.update(
                    rospy.get_param('/motionplanner/environment/room_center'))

        # Topics
        self.trailer_pose3d_pub = rospy.Publisher(
            '/controller/pose_est_trailer', PoseStamped, queue_size=1)
        self.truck_pose3d_pub = rospy.Publisher(
            '/controller/pose_est_truck', PoseStamped, queue_size=1)
        self.obst_pub = rospy.Publisher(
            '/obstacle_detection/rviz_obst', MarkerArray, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            '/motionplanner/desired_path', Marker, queue_size=1)
        self.trajectory_desired_fine = rospy.Publisher(
            '/motionplanner/desired_fine_path', Marker, queue_size=1)
        self.trajectory_toggle = rospy.Publisher(
            '/motionplanner/desired_toggle_path', Marker, queue_size=1)
        self.stage1_path_pub = rospy.Publisher(
            '/motionplanner/stage1_path', Marker, queue_size=1)
        self.stage2_path_pub = rospy.Publisher(
            '/motionplanner/stage2_path', Marker, queue_size=1)
        self.stage3_path_pub = rospy.Publisher(
            '/motionplanner/stage3_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            '/motionplanner/real_path', Marker, queue_size=1)
        self.trailer_ff_vel_pub = rospy.Publisher(
            '/controller/trailer_ff_vel', Marker, queue_size=1)
        self.trailer_fb_vel_pub = rospy.Publisher(
            '/controller/trailer_fb_vel', Marker, queue_size=1)
        self.truck_ff_vel_pub = rospy.Publisher(
            '/controller/truck_ff_vel', Marker, queue_size=1)
        self.truck_fb_vel_pub = rospy.Publisher(
            '/controller/truck_fb_vel', Marker, queue_size=1)
        self.draw_room_pub = rospy.Publisher(
            '/motionplanner/room_contours', Marker, queue_size=1)
        self.active_route_pub = rospy.Publisher(
            '/controller/active_route', MarkerArray, queue_size=1)
        self.active_corridors_pub = rospy.Publisher(
            '/controller/active_corridors', MarkerArray, queue_size=1)
        self.all_routes_pub = rospy.Publisher(
            '/controller/all_routes', MarkerArray, queue_size=1)
        self.corridor_targets_pub = rospy.Publisher(
            '/controller/corridor_targets', MarkerArray, queue_size=1)
        self.vehicle_x0_pub = rospy.Publisher(
            '/controller/vehicle_x0', MarkerArray, queue_size=1)
        self.corridor_checkpoint_pub = rospy.Publisher(
            '/controller/corridor_checkpoint', PointStamped, queue_size=1)

        rospy.sleep(0.5)

    def _marker_setup(self):
        '''Setup markers to display the desired and real path of the AMR in
        rviz, along with the current position in the mp generated
        position list.
        '''
        # Obstacles
        self.rviz_obst = MarkerArray()

        # Corridors
        self.rviz_active_route = MarkerArray()
        self.rviz_all_corridors = MarkerArray()
        self.rviz_active_corridors = MarkerArray()
        self.rviz_corridor_targets = MarkerArray()
        self.rviz_vehicle_x0 = MarkerArray()

        # Desired path
        self.desired_path = Marker()
        self.desired_path.header.frame_id = "map"
        self.desired_path.ns = "trajectory_desired"
        self.desired_path.id = 0
        self.desired_path.type = 4  # Line Strip
        self.desired_path.action = 0
        self.desired_path.scale.x = 0.03
        self.desired_path.color.r = 1.0
        self.desired_path.color.g = 0.0
        self.desired_path.color.b = 0.0
        self.desired_path.color.a = 1.0
        self.desired_path.lifetime = rospy.Duration(0)

        # Desired path fine
        self.desired_fine_path = Marker()
        self.desired_fine_path.header.frame_id = "map"
        self.desired_fine_path.ns = "trajectory_desired_fine"
        self.desired_fine_path.id = 0
        self.desired_fine_path.type = 8  # Sphere List
        self.desired_fine_path.action = 0
        self.desired_fine_path.scale.x = 0.02
        self.desired_fine_path.scale.y = 0.08
        self.desired_fine_path.scale.z = 0.02
        self.desired_fine_path.color.r = 1.0
        self.desired_fine_path.color.g = 1.0
        self.desired_fine_path.color.b = 0.0
        self.desired_fine_path.color.a = 1.0
        self.desired_fine_path.lifetime = rospy.Duration(0)

        # Toggle
        self.desired_toggle_path = Marker()
        self.desired_toggle_path.header.frame_id = "map"
        self.desired_toggle_path.ns = "trajectory_toggle"
        self.desired_toggle_path.id = 0
        self.desired_toggle_path.type = 8  # Sphere List
        self.desired_toggle_path.action = 0
        self.desired_toggle_path.scale.x = 0.02
        self.desired_toggle_path.scale.y = 0.1
        self.desired_toggle_path.scale.z = 0.02
        self.desired_toggle_path.color.r = 1.0
        self.desired_toggle_path.color.g = 0.0
        self.desired_toggle_path.color.b = 1.0
        self.desired_toggle_path.color.a = 1.0
        self.desired_toggle_path.lifetime = rospy.Duration(0)

        # Real path
        self.real_path = Marker()
        self.real_path.header.frame_id = "map"
        self.real_path.ns = "trajectory_real"
        self.real_path.id = 0
        self.real_path.type = 4  # Line Strip
        self.real_path.action = 0
        self.real_path.scale.x = 0.03
        self.real_path.color.r = 0.0
        self.real_path.color.g = 1.0
        self.real_path.color.b = 0.0
        self.real_path.color.a = 1.0
        self.real_path.lifetime = rospy.Duration(0)

        # Room contours
        self.room_contours = Marker()
        self.room_contours.header.frame_id = "map"
        self.room_contours.ns = "room_contours"
        self.room_contours.id = 0
        self.room_contours.type = 4  # Line Strip
        self.room_contours.action = 0
        self.room_contours.scale.x = 0.015
        self.room_contours.color.r = 0.8
        self.room_contours.color.g = 0.8
        self.room_contours.color.b = 0.8
        self.room_contours.color.a = 1.0
        self.room_contours.lifetime = rospy.Duration(0)

        # Feedforward trailer velocity vector
        self.trailer_ff_vel = Marker()
        self.trailer_ff_vel.header.frame_id = "map"
        self.trailer_ff_vel.ns = "trailer_ff_vel"
        self.trailer_ff_vel.id = 0
        self.trailer_ff_vel.type = 0  # Arrow
        self.trailer_ff_vel.action = 0
        self.trailer_ff_vel.scale.x = 0.06  # shaft diameter
        self.trailer_ff_vel.scale.y = 0.1  # head diameter
        self.trailer_ff_vel.scale.z = 0.15  # head length
        self.trailer_ff_vel.color.r = 1.0
        self.trailer_ff_vel.color.g = 1.0
        self.trailer_ff_vel.color.b = 0.3
        self.trailer_ff_vel.color.a = 1.0
        self.trailer_ff_vel.lifetime = rospy.Duration(0)

        # Feedback trailer velocity vector
        self.trailer_fb_vel = Marker()
        self.trailer_fb_vel.header.frame_id = "map"
        self.trailer_fb_vel.ns = "trailer_fb_vel"
        self.trailer_fb_vel.id = 0
        self.trailer_fb_vel.type = 0  # Arrow
        self.trailer_fb_vel.action = 0
        self.trailer_fb_vel.scale.x = 0.06  # shaft diameter
        self.trailer_fb_vel.scale.y = 0.1  # head diameter
        self.trailer_fb_vel.scale.z = 0.15  # head length
        self.trailer_fb_vel.color.r = 0.01
        self.trailer_fb_vel.color.g = 0.01
        self.trailer_fb_vel.color.b = 1.0
        self.trailer_fb_vel.color.a = 1.0
        self.trailer_fb_vel.lifetime = rospy.Duration(0)

        # Feedforward truck velocity vector
        self.truck_ff_vel = Marker()
        self.truck_ff_vel.header.frame_id = "map"
        self.truck_ff_vel.ns = "truck_ff_vel"
        self.truck_ff_vel.id = 0
        self.truck_ff_vel.type = 0  # Arrow
        self.truck_ff_vel.action = 0
        self.truck_ff_vel.scale.x = 0.06  # shaft diameter
        self.truck_ff_vel.scale.y = 0.1  # head diameter
        self.truck_ff_vel.scale.z = 0.15  # head length
        self.truck_ff_vel.color.r = 1.0
        self.truck_ff_vel.color.g = 1.0
        self.truck_ff_vel.color.b = 0.0
        self.truck_ff_vel.color.a = 1.0
        self.truck_ff_vel.lifetime = rospy.Duration(0)

        # Feedback truck velocity vector
        self.truck_fb_vel = Marker()
        self.truck_fb_vel.header.frame_id = "map"
        self.truck_fb_vel.ns = "truck_fb_vel"
        self.truck_fb_vel.id = 0
        self.truck_fb_vel.type = 0  # Arrow
        self.truck_fb_vel.action = 0
        self.truck_fb_vel.scale.x = 0.06  # shaft diameter
        self.truck_fb_vel.scale.y = 0.1  # head diameter
        self.truck_fb_vel.scale.z = 0.15  # head length
        self.truck_fb_vel.color.r = 0.01
        self.truck_fb_vel.color.g = 1.0
        self.truck_fb_vel.color.b = 0.01
        self.truck_fb_vel.color.a = 1.0
        self.truck_fb_vel.lifetime = rospy.Duration(0)

        # Stage 1 path
        self.stage1_path = Marker()
        self.stage1_path.header.frame_id = "map"
        self.stage1_path.ns = "stage_1_traj"
        self.stage1_path.id = 0
        self.stage1_path.type = 4  # Line Strip
        self.stage1_path.action = 0
        self.stage1_path.scale.x = 0.03
        self.stage1_path.color.r = 1.0
        self.stage1_path.color.g = 0.0
        self.stage1_path.color.b = 0.0
        self.stage1_path.color.a = 1.0
        self.stage1_path.lifetime = rospy.Duration(0)

        # Stage 2 path
        self.stage2_path = Marker()
        self.stage2_path.header.frame_id = "map"
        self.stage2_path.ns = "stage_2_traj"
        self.stage2_path.id = 0
        self.stage2_path.type = 4  # Line Strip
        self.stage2_path.action = 0
        self.stage2_path.scale.x = 0.03
        self.stage2_path.color.r = 1.0
        self.stage2_path.color.g = 1.0
        self.stage2_path.color.b = 0.0
        self.stage2_path.color.a = 1.0
        self.stage2_path.lifetime = rospy.Duration(0)

        # Stage 3 path
        self.stage3_path = Marker()
        self.stage3_path.header.frame_id = "map"
        self.stage3_path.ns = "stage_3_traj"
        self.stage3_path.id = 0
        self.stage3_path.type = 4  # Line Strip
        self.stage3_path.action = 0
        self.stage3_path.scale.x = 0.03
        self.stage3_path.color.r = 0.0
        self.stage3_path.color.g = 0.0
        self.stage3_path.color.b = 1.0
        self.stage3_path.color.a = 1.0
        self.stage3_path.lifetime = rospy.Duration(0)

    def reset_traj_markers(self):
        '''Resets all Rviz markers (except for obstacles and corridors).
        '''
        self.desired_path.points = []
        self.desired_fine_path.points = []
        self.desired_toggle_path.points = []
        self.stage1_path.points = []
        self.stage2_path.points = []
        self.stage3_path.points = []
        self.real_path.points = []
        self.trailer_ff_vel.points = [Point(), Point()]
        self.trailer_fb_vel.points = [Point(), Point()]
        self.truck_ff_vel.points = [Point(), Point()]
        self.truck_fb_vel.points = [Point(), Point()]

        self.trajectory_desired.publish(self.desired_path)
        self.trajectory_desired_fine.publish(self.desired_fine_path)
        self.trajectory_toggle.publish(self.desired_toggle_path)
        self.stage1_path_pub.publish(self.stage1_path)
        self.stage2_path_pub.publish(self.stage2_path)
        self.stage3_path_pub.publish(self.stage3_path)
        self.trajectory_real.publish(self.real_path)
        self.trailer_ff_vel_pub.publish(self.trailer_ff_vel)
        self.trailer_fb_vel_pub.publish(self.trailer_fb_vel)
        self.truck_ff_vel_pub.publish(self.truck_ff_vel)
        self.truck_fb_vel_pub.publish(self.truck_fb_vel)

    def reset_velocity_markers(self):
        '''Resets all velocity markers.
        '''
        self.trailer_ff_vel.points = [Point(), Point()]
        self.trailer_fb_vel.points = [Point(), Point()]
        self.truck_ff_vel.points = [Point(), Point()]
        self.truck_fb_vel.points = [Point(), Point()]

        self.trailer_ff_vel_pub.publish(self.trailer_ff_vel)
        self.trailer_fb_vel_pub.publish(self.trailer_fb_vel)
        self.truck_ff_vel_pub.publish(self.truck_ff_vel)
        self.truck_fb_vel_pub.publish(self.truck_fb_vel)

    def publish_truck_trailer_poses(self, truck_pose2d, trailer_pose2d):
        '''Publish the poses of the truck and trailer in 3D for visualization
        in Rviz.
        '''
        truck_pose3D = PoseStamped()
        truck_pose3D.header.stamp = rospy.Time.now()
        truck_pose3D.header.frame_id = "map"
        truck_pose3D.pose.position.x = truck_pose2d.x
        truck_pose3D.pose.position.y = truck_pose2d.y
        truck_pose3D.pose.orientation = Quaternion(
                    *helpers.quaternion_from_euler(0, 0, truck_pose2d.theta))
        trailer_pose3D = PoseStamped()
        trailer_pose3D.header.stamp = rospy.Time.now()
        trailer_pose3D.header.frame_id = "map"
        trailer_pose3D.pose.position.x = trailer_pose2d.x
        trailer_pose3D.pose.position.y = trailer_pose2d.y
        trailer_pose3D.pose.orientation = Quaternion(
                    *helpers.quaternion_from_euler(0, 0, trailer_pose2d.theta))

        self.trailer_pose3d_pub.publish(trailer_pose3D)
        self.truck_pose3d_pub.publish(truck_pose3D)

    def publish_desired(self, x_traj, y_traj, z_traj):
        '''Publish planned x and y trajectory to topic for visualization in
        rviz.

        :param x_traj: x trajectory to be plotted
        :type x_traj: float[]

        :param y_traj: y trajectory to be plotted
        :type y_traj: float[]

        :param z_traj: z trajectory to be plotted
        :type z_traj: float[]
        '''
        self.desired_path.points = \
            [Point(x=x_traj[k], y=y_traj[k]) for k in range(len(x_traj))]
        self.trajectory_desired.publish(self.desired_path)

    def publish_desired_fine(self, x_traj, y_traj, z_traj):
        '''Publish planned x and y trajectory to topic for visualization in
        rviz.

        :param x_traj: x trajectory to be plotted
        :type x_traj: float[]

        :param y_traj: y trajectory to be plotted
        :type y_traj: float[]

        :param z_traj: z trajectory to be plotted
        :type z_traj: float[]
        '''
        self.desired_fine_path.points = \
            [Point(x=x_traj[k], y=y_traj[k]) for k in range(len(x_traj))]
        self.trajectory_desired_fine.publish(self.desired_fine_path)

    def publish_toggle_check(self, x_traj, y_traj, z_traj):
        '''Publish x and y trajectory at which possible corridor toggle is
        checked to topic for visualization in rviz.

        :param x_traj: x trajectory to be plotted
        :type x_traj: float[]

        :param y_traj: y trajectory to be plotted
        :type y_traj: float[]

        :param z_traj: z trajectory to be plotted
        :type z_traj: float[]
        '''
        self.desired_toggle_path.points = \
            [Point(x=x_traj[k], y=y_traj[k]) for k in range(len(x_traj))]
        self.trajectory_toggle.publish(self.desired_toggle_path)

    def publish_desired_multistage(self, x1_traj, y1_traj, x2_traj, y2_traj,
                                   x3_traj, y3_traj):
        '''Publish planned stages of trajectory to topic for visualization in
        rviz.

        :param x1_traj: x trajectory (stage1) to be plotted
        :type x1_traj: float[]

        :param y1_traj: y trajectory (stage1) to be plotted
        :type y1_traj: float[]

        :param x2_traj: x trajectory (stage2) to be plotted
        :type x2_traj: float[]

        :param y2_traj: y trajectory (stage2) to be plotted
        :type y2_traj: float[]

        :param x3_traj: x trajectory (stage3) to be plotted
        :type x3_traj: float[]

        :param y3_traj: y trajectory (stage3) to be plotted
        :type y3_traj: float[]t
        '''
        # Stage 1
        self.stage1_path.points = []
        self.stage1_path_pub.publish(self.stage1_path)
        self.stage1_path.header.stamp = rospy.Time(0)

        stage1_pos = [0]*len(x1_traj)
        for k in range(len(x1_traj)):
            stage1_pos[k] = Point(x=x1_traj[k], y=y1_traj[k])
        self.stage1_path.points = stage1_pos

        # Stage 2
        self.stage2_path.points = []
        self.stage2_path_pub.publish(self.stage2_path)
        self.stage2_path.header.stamp = rospy.Time(0)

        stage2_pos = [0]*len(x2_traj)
        for k in range(len(x2_traj)):
            stage2_pos[k] = Point(x=x2_traj[k], y=y2_traj[k])
        self.stage2_path.points = stage2_pos

        # Stage 3
        self.stage3_path.points = []
        self.stage3_path_pub.publish(self.stage3_path)
        self.stage3_path.header.stamp = rospy.Time(0)

        stage3_pos = [0]*len(x3_traj)
        for k in range(len(x3_traj)):
            stage3_pos[k] = Point(x=x3_traj[k], y=y3_traj[k])
        self.stage3_path.points = stage3_pos

        self.stage1_path_pub.publish(self.stage1_path)
        self.stage2_path_pub.publish(self.stage2_path)
        self.stage3_path_pub.publish(self.stage3_path)

    def publish_real(self, x_pos, y_pos, z_pos):
        '''Publish real x and y trajectory to topic for visualization in
        rviz.

        :param x_pos: x position
        :type x_pos: float

        :param y_pos: y position
        :type y_pos: float

        :param z_pos: z position
        :type z_pos: float
        '''
        self.real_path.header.stamp = rospy.Time(0)

        point = Point(x=x_pos, y=y_pos, z=z_pos)
        # After a while list becomes really long so only keep last XXXX values.
        if len(self.real_path.points) > 1000:
            self.real_path.points = self.real_path.points[1:] + [point]
        else:
            self.real_path.points.append(point)

        self.trajectory_real.publish(self.real_path)

    def publish_trailer_velocities(self, pose1, v1_fb):
        '''Publish current trailer velocities where origin of the
        vector is equal to current mp position.

        :param pose1: trailer pose
        :type pose1: geometry_msgs.msg.Pose2D

        :param v1_fb: trailer feedback velocity - [vx, vy]
        :type v1_fb: np.array
        '''
        scale_vec = 50

        # point_start = Point(*pos1)
        # point_end = Point(*(pos1 + v1_ff*scale_vec))
        # self.trailer_ff_vel.points = [point_start, point_end]
        self.trailer_fb_vel.header.stamp = rospy.Time(0)
        point_start = Point(x=pose1.x,
                            y=pose1.y)
        point_end = Point(x=pose1.x + v1_fb[0]*scale_vec,
                          y=pose1.y + v1_fb[1]*scale_vec)
        self.trailer_fb_vel.points = [point_start, point_end]

        # self.trailer_ff_vel_pub.publish(self.trailer_ff_vel)
        self.trailer_fb_vel_pub.publish(self.trailer_fb_vel)

    def publish_truck_velocities(self, pose0, v0_ff, v0_fb):
        '''Publish current truck velocities where origin of the
        vector is equal to current mp position.

        :param pose0: trailer pose
        :type pose0: geometry_msgs.msg.Pose2D

        :param v0_ff: truck feedforward velocity - [vx, vy]
        :type v0_ff: np.array

        :param v0_fb: truck feedback velocity - [vx, vy]
        :type v0_fb: np.array

        '''
        scale_vec = 50

        self.truck_ff_vel.header.stamp = rospy.Time(0)
        point_start = Point(x=pose0.x,
                            y=pose0.y)
        point_end = Point(x=pose0.x + v0_ff[0]*scale_vec,
                          y=pose0.y + v0_ff[1]*scale_vec)
        self.truck_ff_vel.points = [point_start, point_end]

        self.truck_fb_vel.header.stamp = rospy.Time(0)
        point_start = Point(x=pose0.x,
                            y=pose0.y)
        point_end = Point(x=pose0.x + v0_fb[0]*scale_vec,
                          y=pose0.y + v0_fb[1]*scale_vec)
        self.truck_fb_vel.points = [point_start, point_end]

        self.truck_ff_vel_pub.publish(self.truck_ff_vel)
        self.truck_fb_vel_pub.publish(self.truck_fb_vel)

    def draw_all_routes(self, routes):
        '''Visualize complete route.

        :param route: route = sequence of corridors
        :type route: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        for i, corridor in enumerate(routes):
            # Marker setup
            corridor_marker = Marker()
            corridor_marker.header.stamp = rospy.Time.now()
            corridor_marker.header.frame_id = "map"
            corridor_marker.ns = "all_routes"
            corridor_marker.id = i
            corridor_marker.action = 0
            corridor_marker.scale.x = 0.02
            corridor_marker.color.r = 0.5
            corridor_marker.color.g = 0.
            corridor_marker.color.b = 0.
            corridor_marker.color.a = 0.9
            corridor_marker.pose.orientation.w = 1
            corridor_marker.lifetime = rospy.Duration(0)
            corridor_marker.type = 4  # Line Strip
            corridor_marker.points = corridor.corners + [corridor.corners[0]]

            # Append marker to marker array:
            self.rviz_all_corridors.markers.append(corridor_marker)

        self.all_routes_pub.publish(self.rviz_all_corridors)

    def draw_active_route(self, route):
        '''Publish corridors of active route for visualization.

        :param route: route = sequence of corridors
        :type route: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        # Delete markers
        marker = Marker()
        marker.ns = "active_route"
        marker.header.frame_id = "map"
        marker.action = marker.DELETEALL
        self.rviz_active_route.markers = [marker]
        self.active_route_pub.publish(self.rviz_active_route)

        for i, corridor in enumerate(route):
            # Marker setup
            corridor_marker = Marker()
            corridor_marker.header.stamp = rospy.Time.now()
            corridor_marker.header.frame_id = "map"
            corridor_marker.ns = "active_route"
            corridor_marker.id = i
            corridor_marker.action = 0
            corridor_marker.scale.x = 0.05
            corridor_marker.color.r = 0.1
            corridor_marker.color.g = 0.4
            corridor_marker.color.b = 0.6
            corridor_marker.color.a = 1.0
            corridor_marker.pose.orientation.w = 1
            corridor_marker.lifetime = rospy.Duration(0)
            corridor_marker.type = 4  # Line Strip
            corridor_marker.points = corridor.corners + [corridor.corners[0]]

            # Append marker to marker array:
            self.rviz_active_route.markers.append(corridor_marker)

        self.active_route_pub.publish(self.rviz_active_route)

    def draw_active_corridors(self, route):
        '''Publish active corridors for visualization.

        :param route: route = sequence of corridors
        :type route: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        # Delete markers
        marker = Marker()
        marker.ns = "active_corridors"
        marker.header.frame_id = "map"
        marker.action = marker.DELETEALL
        self.rviz_active_corridors.markers = [marker]
        self.active_corridors_pub.publish(self.rviz_active_corridors)

        for i, corridor in enumerate(route):
            # Marker setup
            corridor_marker = Marker()
            corridor_marker.header.stamp = rospy.Time.now()
            corridor_marker.header.frame_id = "map"
            corridor_marker.ns = "active_corridors"
            corridor_marker.id = i
            corridor_marker.action = 0
            corridor_marker.scale.x = 0.09
            corridor_marker.color.r = 0.9
            corridor_marker.color.g = 0.6
            corridor_marker.color.b = 0.1
            corridor_marker.color.a = 1.0
            corridor_marker.pose.orientation.w = 1
            corridor_marker.lifetime = rospy.Duration(0)
            corridor_marker.type = 4  # Line Strip
            corridor_marker.points = corridor.corners + [corridor.corners[0]]

            # Append marker to marker array:
            self.rviz_active_corridors.markers.append(corridor_marker)

        self.active_corridors_pub.publish(self.rviz_active_corridors)

    def draw_corridor_targets(self, route):
        '''Publish corridor targets for visualization.

        :param route: route = sequence of corridors
        :type route: truck_trailer_amr.corridor_handler.Corridor[]
        '''
        # Delete markers
        marker = Marker()
        marker.ns = "corridor_targets"
        marker.header.frame_id = "map"
        marker.action = marker.DELETEALL
        self.rviz_corridor_targets.markers = [marker]
        self.corridor_targets_pub.publish(self.rviz_corridor_targets)

        self.rviz_corridor_targets = MarkerArray()

        for i, corridor in enumerate(route):
            # Marker setup
            trailer_marker = Marker()
            trailer_marker.header.stamp = rospy.Time.now()
            trailer_marker.header.frame_id = "map"
            trailer_marker.ns = "corridor_targets"
            trailer_marker.id = i
            trailer_marker.action = 0
            trailer_marker.scale.x = 0.03
            trailer_marker.color.r = 0.
            trailer_marker.color.g = 0.7
            trailer_marker.color.b = 0.
            trailer_marker.color.a = 1.0
            trailer_marker.pose.orientation.w = 1
            trailer_marker.lifetime = rospy.Duration(0)
            trailer_marker.type = 4  # Line Strip
            trailer_marker.points = \
                corridor.trailer_corners + [corridor.trailer_corners[0]]

            # Append marker to marker array:
            self.rviz_corridor_targets.markers.append(trailer_marker)

        for i, corridor in enumerate(route):
            # Marker setup
            truck_marker = Marker()
            truck_marker.header.stamp = rospy.Time.now()
            truck_marker.header.frame_id = "map"
            truck_marker.ns = "corridor_targets"
            truck_marker.id = 100 + i
            truck_marker.action = 0
            truck_marker.scale.x = 0.03
            truck_marker.color.r = 0.
            truck_marker.color.g = 0.7
            truck_marker.color.b = 0.
            truck_marker.color.a = 1.0
            truck_marker.pose.orientation.w = 1
            truck_marker.lifetime = rospy.Duration(0)
            truck_marker.type = 4  # Line Strip
            truck_marker.points = \
                corridor.truck_corners + [corridor.truck_corners[0]]

            # Append marker to marker array:
            self.rviz_corridor_targets.markers.append(truck_marker)

        self.corridor_targets_pub.publish(self.rviz_corridor_targets)

    def draw_vehicles_x0(self, trailer, truck):
        '''Publish vehicles x0 for visualization.

        :param trailer: Trailer vehicle vertices
        :type trailer: geometry_msgs.msg.Point[]
        :param truck: Truck vehicle vertices
        :type truck: geometry_msgs.msg.Point[]
        '''
        # Delete markers
        marker = Marker()
        marker.ns = "vehicles_x0"
        marker.header.frame_id = "map"
        marker.action = marker.DELETEALL
        self.rviz_vehicle_x0.markers = [marker]
        self.vehicle_x0_pub.publish(self.rviz_vehicle_x0)

        self.rviz_vehicle_x0 = MarkerArray()

        # Marker setup
        trailer_marker = Marker()
        trailer_marker.header.stamp = rospy.Time.now()
        trailer_marker.header.frame_id = "map"
        trailer_marker.ns = "vehicles_x0"
        trailer_marker.id = 0
        trailer_marker.action = 0
        trailer_marker.scale.x = 0.03
        trailer_marker.color.r = 0.7
        trailer_marker.color.g = 0.
        trailer_marker.color.b = 0.
        trailer_marker.color.a = 1.0
        trailer_marker.pose.orientation.w = 1
        trailer_marker.lifetime = rospy.Duration(0)
        trailer_marker.type = 4  # Line Strip
        trailer_marker.points = trailer + [trailer[0]]

        # Append marker to marker array:
        self.rviz_vehicle_x0.markers.append(trailer_marker)

        # Marker setup
        truck_marker = Marker()
        truck_marker.header.stamp = rospy.Time.now()
        truck_marker.header.frame_id = "map"
        truck_marker.ns = "vehicles_x0"
        truck_marker.id = 1
        truck_marker.action = 0
        truck_marker.scale.x = 0.03
        truck_marker.color.r = 0.
        truck_marker.color.g = 0.
        truck_marker.color.b = 0.7
        truck_marker.color.a = 1.0
        truck_marker.pose.orientation.w = 1
        truck_marker.lifetime = rospy.Duration(0)
        truck_marker.type = 4  # Line Strip
        truck_marker.points = truck + [truck[0]]

        # Append marker to marker array:
        self.rviz_vehicle_x0.markers.append(truck_marker)

        self.vehicle_x0_pub.publish(self.rviz_vehicle_x0)

    def draw_room_contours(self):
        '''Publish the edges of the room for visualization in rviz.
        '''
        self.room_contours.header.stamp = rospy.Time(0)
        w = self.room["width"]
        d = self.room["depth"]
        x_c = self.room["x"]
        y_c = self.room["y"]

        bottom_left = Point(x=-w/2.+x_c, y=-d/2.+y_c)
        bottom_right = Point(x=w/2.+x_c, y=-d/2.+y_c)
        top_right = Point(x=w/2.+x_c, y=d/2.+y_c)
        top_left = Point(x=-w/2.+x_c, y=d/2.+y_c)
        corners = [bottom_left, bottom_right, top_right, top_left, bottom_left]
        for point in corners:
            self.room_contours.points.append(point)

        self.draw_room_pub.publish(self.room_contours)
