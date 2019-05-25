#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
import math
import numpy as np
from math import radians

import PathBuilder
import rospy
import tf
import tf2_ros
from Agent import Agent
from GridPOI import GridPOI
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from bresenham import bresenham
from Grid import m_to_cm

# Drone position
class drone_pos_goal:
    def __init__(self, time=None, pos=None, goal=None, yaw=None):
        self.time = time
        self.pos = pos
        self.goal = goal
        self.yaw = yaw


def to_pose_stamped(x, y, z, roll, pitch, yaw):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # quaternion = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
    # yaw is already in radians
    quaternion = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose


def is_los(p1, p2, matrix, x_lim, y_lim, res):
    n = int(np.maximum(1, np.ceil(np.linalg.norm(np.subtract(p1, p2)) / res) * 3))
    x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
    y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
    for ind in range(1, n):
        i, j = xy_to_ij(x[ind], y[ind], x_lim, y_lim, res)
        if matrix[i][j] != 1:
            return False
    return True


def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0]) / res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j


# def get_goal_point(pos, interesting_points_list_xy, matrix, x_lim, y_lim, res, paths_to_wps, min_dist_between_goals):
#     g_idx = []
#     dist_arr = []
#
#     for ielem, elem in enumerate(interesting_points_list_xy):
#         dist_arr.append(np.linalg.norm(np.subtract(elem, pos[0])))
#     sorted_dist_idxs = sorted(range(len(dist_arr)), key=lambda k: dist_arr[k])
#     if paths_to_wps != []:
#         for idx in sorted_dist_idxs:
#             i_s, j_s = xy_to_ij(pos[0][0], pos[0][1], x_lim, y_lim, res)
#             i_g, j_g = xy_to_ij(interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1], x_lim, y_lim, res)
#             path_curr_sg = list(bresenham(i_s, j_s, i_g, j_g))
#             valid_wp = True
#             overlapping_tails = []
#
#             for path_sg in paths_to_wps:
#                 # overlapping_tails = set(path_curr_sg).intersection(path_sg)
#                 overlapping_tails = list(set(path_curr_sg).intersection(set(path_sg)))
#                 dist_between_goals = np.linalg.norm(np.subtract(list(path_sg[-1]), list(path_curr_sg[-1])))
#                 if overlapping_tails != [] and dist_between_goals < min_dist_between_goals:
#                     valid_wp = False
#                     break
#
#             if valid_wp == True:
#                 g_idx = idx
#                 break
#     else:
#         # g_idx = 0
#         g_idx = g_idx = np.random.randint(len(interesting_points_list_xy))
#         for idx in sorted_dist_idxs:
#             if is_los(pos, [[interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1]]], matrix, x_lim, y_lim,
#                       res):
#                 g_idx = idx
#                 break
#
#     return g_idx


def get_goal_point(pos, interesting_points_list_xy, x_lim, y_lim, res, drones_pos_list, n_tails_between_drones, tf_prefix):
    valid_wp_flag = True
    goal = pos
    g_idx = []
    i_s, j_s = xy_to_ij(pos[0][0], pos[0][1], x_lim, y_lim, res)
    for i_elem, elem in enumerate(interesting_points_list_xy):
        i_g, j_g = xy_to_ij(elem[0], elem[1], x_lim, y_lim, res)
        path_curr_sg = list(bresenham(i_s, j_s, i_g, j_g))
        for i_prefix, prefix in enumerate(drones_pos_list):
            if prefix != tf_prefix:
                next_drone_pos = drones_pos_list[prefix].pos
                next_drone_goal = drones_pos_list[prefix].goal
                i_p_s, j_p_s = xy_to_ij(next_drone_pos[0], next_drone_pos[1], x_lim, y_lim, res)
                i_p_g, j_p_g = xy_to_ij(next_drone_goal[0], next_drone_goal[1], x_lim, y_lim, res)
                if np.linalg.norm(np.subtract([i_g, j_g], [i_p_s, j_p_s])) > n_tails_between_drones:
                    path_p_sg = list(bresenham(i_p_s, j_p_s, i_p_g, j_p_g))
                    overlapping_tails = list(set(path_curr_sg).intersection(set(path_p_sg)))
                    if overlapping_tails != []:
                        valid_wp_flag = False
                        break
                else:
                    break
        if valid_wp_flag == True:
            g_idx = i_elem
            goal = elem

    return g_idx, goal


class DroneCjInjector:
    """DroneCjInjector:
                parameters:
                    drone_name,
                    drone_init_takeoff_input,
                    env_limits - passed to Agent.

                methods:
                    update_pos:
                        recieves new pos, and start the pathfinder algorithem pipeline.

                publish:
                    Cj_injector - the next point to investigate, after it has been preproccesed.

    """

    def __init__(self, drone_name, drone_init_takeoff_input, env_limits, matrix, res):
        # crazyflie name
        self.tf_prefix = drone_name
        self.env_limits = env_limits
        self.res = res
        x = drone_init_takeoff_input[0] * m_to_cm
        y = drone_init_takeoff_input[1] * m_to_cm
        z = drone_init_takeoff_input[2] * m_to_cm
        self.pos = [x, y, z, 0, 0, 0]
        self.next_pose = self.pos
        self.agent = Agent(self.tf_prefix, [self.pos[0:2]], self.res, self.env_limits)
        self.pathfinder = None
        self.matrix = matrix
        self.drone_yaw = math.radians(0)
        self.rot_enabled = False
        self.rot_time_thresh = 10  # sec
        self.last_time_rot_called = rospy.Time.now().to_sec()
        # Init publisher
        self.Cj_injector_pub = rospy.Publisher('/' + self.tf_prefix + "/Cj_injcetor", PoseStamped,
                                               queue_size=1)  # hover message publisher

    def update_pos(self, drone_pos, new_matrix, next_point, drone_yaw, corrners_array, act_as_flag):
        """This function is being called in DroneInjector, every time a pose is recieved for this drone.
            :param
                new_drone_pos - received from Father's subscriber.
                new_matrix - recieved fromFather. same for all drones.
                nex_point - specific for this drone, calculated in Father's POI function
                corrners_array - allowed land marks,
                """
        self.matrix = new_matrix
        self.pos[0] = drone_pos[0][0]
        self.pos[1] = drone_pos[0][1]

        self.rot_enabled = False
        if (rospy.Time.now().to_sec() - self.last_time_rot_called) >= self.rot_time_thresh:
            self.rot_enabled = True
            # temp_yaw = np.random.rand() * np.pi / 4
            self.drone_yaw = drone_yaw + (np.pi / 2)
            # rospy.logdebug("angle {}".format(self.drone_yaw))
            self.drone_yaw = np.mod(self.drone_yaw, np.pi)  # limit the rotation by maximum angle
            # rospy.logdebug("angle after mod {}".format(self.drone_yaw))
            self.last_time_rot_called = rospy.Time.now().to_sec()

        # Assume that new_pos = [x,y,z,r,p,y]
        if act_as_flag:
            Astar_Movement = PathBuilder.build_trj(drone_pos, self.env_limits, self.res, self.matrix, corrners_array,
                                                   next_point)
            self.agent.astar_path = Astar_Movement

        self.agent.preform_step_sys_sim(drone_pos, self.drone_yaw, self.matrix)

        # self.next_pose[0] = self.pos[0]/m_to_cm # Only for debug - inject input to output
        # self.next_pose[1] = self.pos[1]/m_to_cm # Only for debug - inject input to output
        # self.next_pose[5] = 0 # Only for debug

        self.next_pose[0] = self.agent.next_pos[0][0]
        self.next_pose[1] = self.agent.next_pos[0][1]
        self.next_pose[5] = self.drone_yaw
        self.next_pose[2] = 0.35  # hard coded Z height
        # # return to original angle after rotation
        # if self.rot_enabled:
        #     self.drone_yaw = drone_yaw

        x = self.next_pose[0] / m_to_cm
        y = self.next_pose[1] / m_to_cm
        z = self.next_pose[2]
        roll = 0
        pitch = 0
        yaw = self.next_pose[5]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)

        # rospy.logdebug("drone pos: \n{}".format(drone_pos))
        # rospy.logdebug("drone next_point: \n{}".format(next_point))
        self.Cj_injector_pub.publish(pose)


def injector(drone_injector, path):
    """Simple threaded function for publishing point of a drone.

    calling this function, in order to inject poses to a specific drone.


    only AGENT can inject injections.
    """
    for point in path:
        time_delay = point[0]
        x = point[1]
        y = point[2]
        z = point[3]
        roll = 0
        pitch = 0
        yaw = point[4]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        drone_injector.Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)


class DroneInjector:
    """This class will subscribe to Mj topic, and send each drone it's next Cj.

    DroneInjector:
        parametes:
            prefix_list - list of drone names
            num_of_drones
            limits - x_lim, y_lim - the size of matrice that will be extract from GRID
            resolution - tail size

        subscribes:
            grid - (we have an example in agent_0)
            position per drone - (we have an example in agent_0)

        contains:
            DroneCjInjector's - per drone. an object that will publish the next point after inner filtering and
            postprocessing.
    """

    def __init__(self, prefix_takeoff_dict_input, env_limits_input, resolution, rate=15):
        self.cj_injector_container = []
        self.res = resolution
        self.num_of_drones = len(prefix_takeoff_dict_input.keys())  # get number of drones
        self.env_limits = env_limits_input
        self.POI_time_thresh = 5  # sec
        self.last_time_POI_called = rospy.Time.now().to_sec()
        self.interesting_points_list_xy = []
        self.corner_points_list_xy = []
        self.POI_enabled = False
        self.POI = GridPOI(resolution, env_limits_input)
        self.matrix = np.zeros([np.int64(np.ceil((self.env_limits[1] - self.env_limits[0]) / self.res)),
                                np.int64(np.ceil((self.env_limits[3] - self.env_limits[2]) / self.res))])
        # self.matrix = np.zeros([np.int64(np.ceil((self.env_limits[3] - self.env_limits[2]) / self.res)),
        #                         np.int64(np.ceil((self.env_limits[1] - self.env_limits[0]) / self.res))])
        self.drones_pos_list = dict()
        # initiate DroneCjInjector per drone and add it to container
        for drone_pref in prefix_takeoff_dict_input:
            rospy.logdebug("\n\n******************* Initiating DroneCjInjector for drone: {}".format(drone_pref))

            drone_init_takeoff = prefix_takeoff_dict_input[drone_pref]
            drone = DroneCjInjector(drone_pref, drone_init_takeoff, self.env_limits, self.matrix, resolution)
            self.cj_injector_container.append(drone)
            self.drones_pos_list[drone_pref] = drone_pos_goal(time=rospy.Time.now().to_sec(),
                                                              pos=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm],
                                                              goal=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm],
                                                              yaw = None)

        self.rate = rate

        # Init listeners:
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser)

    def grid_parser(self, msg):

        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)
        self.matrix = np.array(msg.data).reshape((grid_width, grid_height))

        # Set timer for POI
        self.POI_enabled = False
        if (rospy.Time.now().to_sec() - self.last_time_POI_called) >= self.POI_time_thresh:
            self.POI_enabled = True
            self.last_time_POI_called = rospy.Time.now().to_sec()
            [_, self.interesting_points_list_xy, _, self.corner_points_list_xy] = self.POI.find_POI(self.matrix)

        x_lim = self.env_limits[0:2]
        y_lim = self.env_limits[2:4]
        self.min_dist_between_drones = 15
        paths_to_wps = []

        for drone in self.cj_injector_container:

        # try:
            trans = self.tfBuffer.lookup_transform('world', drone.tf_prefix, rospy.Time(0))

            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)

            euler = euler_from_quaternion(q, axes='sxyz')

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            pos = [x * m_to_cm, y * m_to_cm, z * m_to_cm, roll, pitch, yaw]
            # rospy.loginfo("pos in Display: {}\n".format(self.pos))
            cur_pos = [[pos[0], pos[1]]]

            if self.POI_enabled and self.interesting_points_list_xy != []:

                g_idx, goal = get_goal_point(cur_pos, self.interesting_points_list_xy, x_lim, y_lim, self.res,
                                       self.drones_pos_list, self.min_dist_between_drones, drone.tf_prefix)

                # if np.random.rand < 0.1:
                #     g_idx = np.random.randint(len(self.interesting_points_list_xy))
                # else:
                #     g_idx = 0
                # gx = self.interesting_points_list_xy[g_idx][0]
                # gy = self.interesting_points_list_xy[g_idx][1]
                # goal = [gx, gy]

                rospy.logdebug("taking goal from get_goal_point {} for drone {}".format(goal, drone.tf_prefix))

                if g_idx != []:
                    del self.interesting_points_list_xy[g_idx]

            else:
                goal = [drone.next_pose[0], drone.next_pose[1]]
                rospy.logdebug("taking position as goal {} for drone {}".format(goal, drone.tf_prefix))

            self.drones_pos_list[drone.tf_prefix] = drone_pos_goal(time=rospy.Time.now().to_sec(),
                                                                   pos=cur_pos[0], goal=goal, yaw=pos[5])

            drone.update_pos(cur_pos, self.matrix, goal, yaw, self.corner_points_list_xy, self.POI_enabled)

            # rospy.logdebug("in grid_parser - drone.update:\n"
            #                "pos: {}\n"
            #                "next_pos: {}\n"
            #                "".format(pos,pos[0:2]))

        # except Exception as e:
        #     rospy.logdebug(e)


if __name__ == '__main__':
    rospy.init_node("incjetor", log_level=rospy.DEBUG)

    prefix_list_from_launch_file = rospy.get_param("~prefix_list")
    initial_takeoff_list_from_launch_file = rospy.get_param("~init_takeoff_list")
    limits_from_launch_file = rospy.get_param("~env_lim")
    res_from_launch_file = rospy.get_param("~resolution")
    env_space = rospy.get_param("~env_space")
    nDrones = rospy.get_param("~nDrones")

    x_lim = (limits_from_launch_file[0][0] - env_space, limits_from_launch_file[0][1] + env_space)
    y_lim = (limits_from_launch_file[0][2] - env_space, limits_from_launch_file[0][3] + env_space)

    limits_from_launch_file = [x_lim[0], x_lim[1], y_lim[0], y_lim[1]]

    prefix_takeoff_dict = dict()

    for iDrone in range(nDrones):
        pref = prefix_list_from_launch_file[iDrone]
        curr_takeoff = initial_takeoff_list_from_launch_file[iDrone]
        prefix_takeoff_dict[pref[0]] = curr_takeoff

    drone_container = DroneInjector(prefix_takeoff_dict, limits_from_launch_file, res_from_launch_file, 15)

    rospy.loginfo("******************* Publish /Cj_injection_rotation_example for starting the example")

    rospy.spin()