#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
import math
from math import radians
from threading import Thread
import PathBuilder
import numpy as np
import rospy
import tf
import tf2_ros
from Agent import Agent
from Grid import m_to_cm
from GridPOI import GridPOI
# from crazyflie_mapping.srv import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


def to_pose_stamped(x, y, z, roll, pitch, yaw):
    """ Simple pose creator out of recieved Cj_injection. """

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose


def injector(drone_injector, path):
    """Simple threaded function for publishing point of a drone.

    * This function is for PREDEFINED path injection only. basely for demonstrations.

    * To use this function publish "/Cj_Example......."

    * If using the Mapper, only AGENT can inject injections.
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


class DronePosGoal:
    """A simple class for holding drone position."""

    def __init__(self, pos=None, next_pos=None, goal=None, yaw=None):
        self.pos = pos
        self.next_pos = next_pos
        self.goal = goal
        self.yaw = yaw


class DroneCjInjector:
    """ DroneCjInjector:
        :param
            drone_name,
            drone_init_takeoff_input,
            env_limits - passed to Agent.

        methods:
            update_pos:
                recieves new pos, and start the pathfinder algorithem pipeline.

        publish:
            Cj_injector - the next point to investigate, after it has been preprocessed.

    """

    def __init__(self, drone_name, drone_init_takeoff_input, env_limits, matrix, res):
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
        self.z_hard_coded_height = 0.35  # hard coded Z height
        # Init publisher
        self.Cj_injector_pub = rospy.Publisher('/' + self.tf_prefix + "/Cj_injcetor", PoseStamped,
                                               queue_size=1)

    def update_pos(self, drone_pos, new_matrix, next_point, drone_yaw, corrners_array, act_as_flag, dict_of_drones_pos,
                   tf_prefix):
        """This function is being called in DroneInjector, every time a pose is received for this drone.
            :param
                new_drone_pos - received from Father's subscriber.
                new_matrix - recieved fromFather. same for all drones.
                nex_point - specific for this drone, calculated in Father's POI function
                corrners_array - allowed land marks,
                """
        self.matrix = new_matrix
        self.pos[0] = drone_pos[0][0]
        self.pos[1] = drone_pos[0][1]

        # Perform rotation
        self.rot_enabled = False
        if (rospy.Time.now().to_sec() - self.last_time_rot_called) >= self.rot_time_thresh:
            self.rot_enabled = True
            self.drone_yaw = drone_yaw + (np.pi / 2)
            rospy.logdebug("in Cj_injector angle {}".format(self.drone_yaw))
            self.drone_yaw = np.mod(self.drone_yaw, np.pi)
            rospy.logdebug("in Cj_injector angle after mod {}".format(self.drone_yaw))
            self.last_time_rot_called = rospy.Time.now().to_sec()

        # Assume that new_pos = [x,y,z,r,p,y]
        if act_as_flag:
            # If POI flag is enabled, then activate Astar algorithm.
            # This algorithm finds the optimal path to goal (interest point of each drone).
            Astar_Movement = PathBuilder.build_trj(drone_pos, self.env_limits, self.res, self.matrix,
                                                   corrners_array,
                                                   next_point)
            # Update the path to goal in Agent
            self.agent.astar_path = Astar_Movement

        # Choose the relevant point from path according to current position
        self.agent.preform_step_sys_sim(drone_pos, self.drone_yaw, self.matrix, dict_of_drones_pos, tf_prefix)

        # Pass relevant next_point to this drone.
        self.next_pose[0] = self.agent.next_pos[0][0]
        self.next_pose[1] = self.agent.next_pos[0][1]
        self.next_pose[5] = self.drone_yaw
        self.next_pose[2] = self.z_hard_coded_height

        x = self.next_pose[0] / m_to_cm
        y = self.next_pose[1] / m_to_cm
        z = self.next_pose[2]
        roll = 0
        pitch = 0
        yaw = self.next_pose[5]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)

        dict_of_drones_pos[tf_prefix].pos = self.agent.current_pos[0]
        dict_of_drones_pos[tf_prefix].next_pos = self.agent.next_pos[0]
        dict_of_drones_pos[tf_prefix].yaw = yaw

        self.Cj_injector_pub.publish(pose)

        return dict_of_drones_pos


class DronesManager:
    """ This class will subscribe to Mj topic, and send each drone it's next Cj.

    DroneInjector:
        :param
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
        # Hold all drones.
        self.cj_injector_container = []
        self.res = resolution
        # Number of drones.
        self.num_of_drones = len(prefix_takeoff_dict_input.keys())
        self.env_limits = env_limits_input
        self.POI_time_thresh = 5  # sec
        self.last_time_POI_called = rospy.Time.now().to_sec()
        self.interesting_points_list_xy = []
        self.corner_points_list_xy = []
        self.POI_enabled = False
        self.min_dist_between_drones = 15  # tails
        self.POI = GridPOI(resolution, env_limits_input)
        self.matrix = np.zeros([np.int64(np.ceil((self.env_limits[1] - self.env_limits[0]) / self.res)),
                                np.int64(np.ceil((self.env_limits[3] - self.env_limits[2]) / self.res))])
        self.drones_pos_list = dict()
        # Initiate DroneCjInjector per drone and add it to container.
        for drone_pref in prefix_takeoff_dict_input:
            rospy.loginfo("\n\n******************* Initiating DroneCjInjector for drone: {}".format(drone_pref))

            drone_init_takeoff = prefix_takeoff_dict_input[drone_pref]
            drone = DroneCjInjector(drone_pref, drone_init_takeoff, self.env_limits, self.matrix, resolution)
            self.cj_injector_container.append(drone)
            self.drones_pos_list[drone_pref] = DronePosGoal(
                pos=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm],
                next_pos=[], goal=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm], yaw=[])

        self.rate = rate

        # Init listeners:
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser)
        # Subscriber for activating predifined path, mainly used in Presentations.
        self.example_predefined_path_subscriber = rospy.Subscriber("/Cj_injection_predefined_path", Empty,
                                                                   self.launcher)

    def grid_parser(self, msg):
        """ Every OccupancyGrid update, loop on all drones in container.

            Get theirs positions from tfBuffer.
            Give them new goal, if POI is not empty, update the drone new_pos in DroneManager.

        """
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

        for drone in self.cj_injector_container:

            # ROS routine for getting pos from tfBuffer.
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
            cur_pos = [[pos[0], pos[1]]]

            if self.POI_enabled and self.interesting_points_list_xy != []:
                g_idx = self.get_goal_point(self.interesting_points_list_xy, self.matrix, x_lim, y_lim, self.res,
                                            self.min_dist_between_drones, self.drones_pos_list, drone.tf_prefix)
                if g_idx is not []:
                    goal = self.interesting_points_list_xy[g_idx]
                    del self.interesting_points_list_xy[g_idx]
                else:
                    goal = self.drones_pos_list[drone.tf_prefix].goal

                self.drones_pos_list[drone.tf_prefix].pos = cur_pos[0]
                self.drones_pos_list[drone.tf_prefix].goal = goal

            self.drones_pos_list = drone.update_pos(cur_pos, self.matrix, self.drones_pos_list[drone.tf_prefix].goal,
                                                    yaw, self.corner_points_list_xy,
                                                    self.POI_enabled, self.drones_pos_list,
                                                    drone.tf_prefix)

    def get_goal_point(self, interesting_points_list_xy, matrix, x_lim, y_lim, res, n_tails_between_drones,
                       drones_pos_list,
                       tf_prefix):
        """ This function chooses specific interest point for each drone.

            It will choose the closest point which is in LOS (not mandatory)
            and is far from the current and next position of all drones.
            If there is no such point it will be taken randomly.
        """

        dist_arr = []
        dist_from_prev_pos = 2

        for ielem, elem in enumerate(interesting_points_list_xy):
            dist_arr.append(np.linalg.norm(np.subtract(elem, drones_pos_list[tf_prefix].pos)))
        sorted_dist_idxs = sorted(range(len(dist_arr)), key=lambda k: dist_arr[k])

        valid_wp_flag = False
        g_idx = np.random.randint(len(interesting_points_list_xy))
        if len(drones_pos_list) > 1:  # For multiple drones
            i_s, j_s = self.xy_to_ij(drones_pos_list[tf_prefix].pos[0], drones_pos_list[tf_prefix].pos[1], x_lim, y_lim,
                                     res)
            for idx in sorted_dist_idxs:
                i_g, j_g = self.xy_to_ij(interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1], x_lim,
                                         y_lim,
                                         res)
                if np.linalg.norm(np.subtract([i_s, j_s], [i_g, j_g])) > dist_from_prev_pos:
                    for i_prefix, prefix in enumerate(drones_pos_list):
                        if prefix != tf_prefix:
                            add_drone_pos = drones_pos_list[prefix].pos
                            i_p_s, j_p_s = self.xy_to_ij(add_drone_pos[0], add_drone_pos[1], x_lim, y_lim, res)
                            if np.linalg.norm(np.subtract([i_g, j_g], [i_p_s, j_p_s])) > n_tails_between_drones:
                                if self.is_los([[drones_pos_list[tf_prefix].pos[0], drones_pos_list[tf_prefix].pos[1]]],
                                               [[interesting_points_list_xy[idx][0],
                                                 interesting_points_list_xy[idx][1]]],
                                               matrix, x_lim,
                                               y_lim, res):
                                    g_idx = idx
                                    valid_wp_flag = True
                                    break
                    if valid_wp_flag is True:
                        break

            if valid_wp_flag is False:

                for idx in sorted_dist_idxs:
                    i_g, j_g = self.xy_to_ij(interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1],
                                             x_lim,
                                             y_lim,
                                             res)
                    if np.linalg.norm(np.subtract([i_s, j_s], [i_g, j_g])) > dist_from_prev_pos:
                        # If inside, it mean that the goal is far enough from current drone pos.
                        for i_prefix, prefix in enumerate(drones_pos_list):
                            # Loop on all other drones, and see if theirs next_pos is not to close to
                            # this drones next_pose, and it is far enough from this next_pos.
                            if prefix != tf_prefix:
                                add_drone_pos = drones_pos_list[prefix].pos
                                i_p_s, j_p_s = self.xy_to_ij(add_drone_pos[0], add_drone_pos[1], x_lim, y_lim, res)
                                add_drone_next_pos = drones_pos_list[prefix].next_pos
                                i_p_g, j_p_g = self.xy_to_ij(add_drone_next_pos[0], add_drone_next_pos[1], x_lim, y_lim,
                                                             res)

                                condition_1 = np.linalg.norm(
                                    np.subtract([i_g, j_g], [i_p_s, j_p_s])) > n_tails_between_drones
                                condition_2 = np.linalg.norm(
                                    np.subtract([i_g, j_g], [i_p_g, j_p_g])) > n_tails_between_drones

                                if condition_1 and condition_2:
                                    # if inside, than the next_pos of this drone is too close to
                                    # some other drone's next_pos.
                                    g_idx = idx
                                    valid_wp_flag = True
                                    break
                        if valid_wp_flag is True:
                            break
        else:
            # For one drone, no line-of-sight and no collision problems.
            g_idx = 0
            for idx in sorted_dist_idxs:
                if self.is_los([[drones_pos_list[tf_prefix].pos[0], drones_pos_list[tf_prefix].pos[1]]],
                               [[interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1]]], matrix,
                               x_lim,
                               y_lim, res):
                    g_idx = idx
                    break

        return g_idx

    def is_los(self, p1, p2, matrix, x_lim, y_lim, res):
        """ This function checks if there is a line of sight between two point on the grid. """

        n = int(np.maximum(1, np.ceil(np.linalg.norm(np.subtract(p1, p2)) / res) * 3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind], x_lim, y_lim, res)
            if matrix[i][j] != 1:
                return False
        return True

    def xy_to_ij(self, x, y, x_lim, y_lim, res):
        i = int(np.floor((x - x_lim[0]) / res))
        j = int(np.floor((y - y_lim[0]) / res))
        return i, j

    # ********************************************************************** Predefined path **************************
    def simple_rectangel_example(self):
        """ Make all drones in DroneInjector to fly in a rectangle.

        Very simple and stupid example for using the injector.
        path = [[duration,x,y,z,yaw], [], ...]
        """

        # todo: move all this reading to read from an excel or a csv.
        threads = []
        step = 0.3048  # one ft in meters
        step_time = 1.07  # this should be 1.0 and is used for speeding up / slowing down tweaks

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame1 = [
            [step_time * 1, 1 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.3125, 2.25 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.3125, 3.5 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.3125, 3.5 * step, 7.75 * step, 0.35, 0],
            [step_time * 1.3125, 3.5 * step, 9 * step, 0.35, 0],
            [step_time * 1.8375, 1.75 * step, 9 * step, 0.35, 0],
            [step_time * 0.525, 1.75 * step, 9.5 * step, 0.35, 0],
            [step_time * 1.05, 2.75 * step, 9.5 * step, 0.35, 0],
            [step_time * 1.3125, 2.75 * step, 10.75 * step, 0.35, 0],
            [step_time * 1.575, 2.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 0.7875, 2 * step, 12.25 * step, 0.35, 0],
            [step_time * 2.1, 2 * step, 14.25 * step, 0.35, 0],
            [step_time * 1.8375, 3.75 * step, 14.25 * step, 0.35, 0],
            [step_time * 2.1, 3.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 1.575, 5.25 * step, 12.25 * step, 0.35, 0],
            [step_time * 8, 6.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 2.1, 8.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 1.05, 8.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 2.1, 10.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 2.1, 12.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 2.1, 14.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 2.1, 16.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 2.1, 18.75 * step, 13.25 * step, 0.35, 0],
            [step_time * 1.5, 18.75 * step, 13.25 * step, 0, 0]
        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame2 = [
            [step_time * 1, 1 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 2.75 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 4.5 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 6.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.3125, 6.25 * step, 4 * step, 0.35, 0],
            [step_time * 1.05, 6.25 * step, 3 * step, 0.35, 0],
            [step_time * 1.575, 4.75 * step, 3 * step, 0.35, 0],
            [step_time * 1.575, 3.25 * step, 3 * step, 0.35, 0],
            [step_time * 1.05, 3.25 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 4.75 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 6.25 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 7.75 * step, 2 * step, 0.35, 0],
            [step_time * 0.7875, 8.5 * step, 2 * step, 0.35, 0],
            [step_time * 1.6601957715884, 7 * step, 2.5 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 4 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 5.5 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 7 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 8.5 * step, 0.35, 0],
            [step_time * 1.48492424049175, 8 * step, 9.5 * step, 0.35, 0],
            [step_time * 2, 8 * step, 11.5 * step, 0.35, 0],
            [step_time * 1.41360576187281, 8.5 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 10.5 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 12.5 * step, 12.75 * step, 0.35, 0],
            [step_time * 1.05, 12.5 * step, 11.75 * step, 0.35, 0],
            [step_time * 1.575, 14 * step, 11.75 * step, 0.35, 0],
            [step_time * 1.575, 15.5 * step, 11.75 * step, 0.35, 0],
            [step_time * 1.575, 17 * step, 11.75 * step, 0.35, 0],
            [step_time * 1, 17 * step, 11.75 * step, 0.35, 0],
            [step_time * 2, 17 * step, 11.75 * step, 0, 0]

        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame3 = [
            [step_time * 1, 0.25 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.8375, 2 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.8375, 3.75 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.8375, 5.5 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 6.5 * step, 0.35, 0],
            [step_time * 1.05, 7 * step, 7.5 * step, 0.35, 0],
            [step_time * 1.8375, 7 * step, 9.25 * step, 0.35, 0],
            [step_time * 2.1, 9 * step, 9.25 * step, 0.35, 0],
            [step_time * 2.1, 11 * step, 9.25 * step, 0.35, 0],
            [step_time * 2.11634265892837, 13 * step, 9 * step, 0.35, 0],
            [step_time * 1.08231522672464, 14 * step, 9.25 * step, 0.35, 0],
            [step_time * 1.575, 14 * step, 7.75 * step, 0.35, 0],
            [step_time * 1.05, 13 * step, 7.75 * step, 0.35, 0],
            [step_time * 1.3125, 11.75 * step, 7.75 * step, 0.35, 0],
            [step_time * 2.1, 11.75 * step, 5.75 * step, 0.35, 0],
            [step_time * 1.8375, 11.75 * step, 4 * step, 0.35, 0],
            [step_time * 1.05, 12.75 * step, 4 * step, 0.35, 0],
            [step_time * 1.05, 12.75 * step, 3 * step, 0.35, 0],
            [step_time * 1.05, 11.75 * step, 3 * step, 0.35, 0],
            [step_time * 1.05, 11.75 * step, 4 * step, 0.35, 0],
            [step_time * 1.575, 11.75 * step, 5.5 * step, 0.35, 0],
            [step_time * 1.575, 11.75 * step, 7 * step, 0.35, 0],
            [step_time * 1.575, 11.75 * step, 8.5 * step, 0.35, 0],
            [step_time * 1.575, 10.25 * step, 8.5 * step, 0.35, 0],
            [step_time * 1.48492424049175, 9.25 * step, 9.5 * step, 0.35, 0],
            [step_time * 1.575, 9.25 * step, 11 * step, 0.35, 0],
            [step_time * 1.91102884593614, 9.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 11.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 13.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 15.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.16463045344927, 17.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 0, 17.75 * step, 12.25 * step, 0, 0]

        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame4 = [
            [step_time * 1, 0.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 2 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 3.75 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 5.5 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 7.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 2.1, 7.25 * step, 7.25 * step, 0.35, 0],
            [step_time * 2.1, 7.25 * step, 9.25 * step, 0.35, 0],
            [step_time * 1.575, 7.25 * step, 10.75 * step, 0.35, 0],
            [step_time * 1.575, 7.25 * step, 12.25 * step, 0.35, 0],
            [step_time * 0.742462120245875, 7.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 1.575, 9.25 * step, 12.75 * step, 0.35, 0],
            [step_time * 9, 11.25 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.1, 13.25 * step, 12.75 * step, 0.35, 0],
            [step_time * 1.8375, 15 * step, 12.75 * step, 0.35, 0],
            [step_time * 1.8375, 16.75 * step, 12.75 * step, 0.35, 0],
            [step_time * 2.625, 18.75 * step, 11.25 * step, 0.35, 0],
            [step_time * 1.8375, 20.5 * step, 11.25 * step, 0.35, 0],
            [step_time * 1.5, 20.5 * step, 11.25 * step, 0, 0]

        ]

        # todo: init all the threads in a for loop over the self.cj_injector_container.
        t1 = Thread(target=injector, args=(self.cj_injector_container[0], self.drone_delay_list[0], path_maze_frame1,))
        threads.append(t1)

        t2 = Thread(target=injector, args=(self.cj_injector_container[1], self.drone_delay_list[1], path_maze_frame2,))
        threads.append(t2)

        t3 = Thread(target=injector, args=(self.cj_injector_container[2], self.drone_delay_list[2], path_maze_frame3,))
        threads.append(t3)

        t4 = Thread(target=injector, args=(self.cj_injector_container[3], self.drone_delay_list[3], path_maze_frame4,))
        threads.append(t4)

        # start all threads.
        for t in threads:
            t.start()

        # stop workers
        for t in threads:
            t.join()

        rospy.loginfo("********************************* Done ***************************")

    def launcher(self, msg):
        """A callback for simple_examples. """
        self.simple_rectangel_example()


def handle_swarm_takeoff(req):
    rospy.logdebug("Returning {}".format(req))
    # return AddTwoIntsResponse(req.a + req.b)
    return req


if __name__ == '__main__':
    rospy.init_node("incjetor", log_level=rospy.INFO)

    # # Initiating service:
    # s = rospy.Service('SwarmTakeoff', SwarmTakeoff, handle_swarm_takeoff)
    # rospy.loginfo("Service SwarmTakeoff Initiated")

    # Get params from ROS launch file.
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

    drone_container = DronesManager(prefix_takeoff_dict, limits_from_launch_file, res_from_launch_file, 15)

    rospy.loginfo("******************* Publish /Cj_injection_predefined_path for starting the example")
    rospy.loginfo("******************* Publish /SwarmTakeoff for starting Autonomous flight")

    rospy.spin()
