#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
import math
from math import radians
import Astar
import numpy as np
import rospy
import tf
import tf2_ros
from Grid import m_to_cm
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


class Agent:
    """ Class for controlling the drone.

        Each drone has This module chooses the next position for each drone using the function Dynam_Search_in_maze

    0 - unexplored area
    1 - explored empty area
    2 - explored area (wall)

    """
    def __init__(self, AgentID, pos, res, env_limits):
        self.ID = AgentID
        self.step_noise_size = 20
        self.stepSizeLimit = 50
        self.next_pos = pos
        self.current_pos = self.next_pos
        self.next_heading = math.radians(0)
        self.current_heading = self.next_heading
        self.wp_path = []
        self.x_lim = env_limits[0:2]
        self.y_lim = env_limits[2:4]
        self.res = res
        self.dist_factor = 1

    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading

    def preform_step_sys_sim(self, current_pos, current_heading, matrix):
        self.update_current_state(current_pos, current_heading)
        self.Dynam_Search_in_maze(matrix)

    def Dynam_Search_in_maze(self, matrix):

        vec = np.zeros(2)
        tails_from_wall = 1
        as_flag = False
        close_wall = False

        # If there are steps left in the path and the next step is in line of sight then choose it.
        if self.wp_path != [] and self.is_step_legal(self.current_pos,
                                                        np.subtract(self.wp_path[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.wp_path[0], self.current_pos[0])
            as_flag = True
        # If there are steps left in the path and the previous step is not finished and still legal then resume the prevoius step
        elif self.wp_path != [] and np.linalg.norm(
                np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size \
                and self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])
        # If there are no steps in path and the previous step is still legal then resume the previous step
        elif self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])

        # Check if the choosen step will be to close to a wall
        if sum(vec) != 0:
            ivec, jvec = self.xy_to_ij(vec[0], vec[1])
            for ti in range(ivec - tails_from_wall, ivec + tails_from_wall + 1):
                for tj in range(jvec - tails_from_wall, jvec - tails_from_wall + 1):
                    if matrix[ti][tj] == 2:
                        close_wall = True
                        break
        else:
            close_wall = True

        # If indeed it is to close to a wall then move in the same direction but stop a few tail before the wall
        if close_wall:
            if np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.res:
                step = np.multiply(np.divide(vec, np.linalg.norm(vec)),
                                   np.linalg.norm(vec) - (tails_from_wall * self.res))
                if (np.linalg.norm(vec) - (tails_from_wall * self.res)) > 0:
                    vec = step

        # Limit the step size to maximum distance
        if np.linalg.norm(vec) > self.stepSizeLimit:
            temp = np.divide(vec, np.linalg.norm(vec))
            vec = np.multiply(temp, self.stepSizeLimit)

        self.next_pos = self.current_pos + vec
        if as_flag and self.wp_path != []:
            del self.wp_path[0]

    def update_sate(self, pos, heading):
        self.current_pos = pos
        self.current_heading = heading

    def is_step_legal(self, curr_pos, step, matrix):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1] and (
                matrix[i][j] == 1 or matrix[i][j] == 3)):
            return False
        return self.is_los(curr_pos, new_pos, matrix)

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y

    def is_los(self, p1, p2, matrix):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1 - p2) / self.res) * 3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if matrix[i][j] != 1:
                return False
        return True



class DronePosGoal:
    """A simple class for holding drone position."""

    def __init__(self, pos=None, next_pos=None, goal=None, yaw=None):
        self.pos = pos
        self.next_pos = next_pos
        self.goal = goal
        self.yaw = yaw


class Local_planner:
    """ Local_planner:
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

    def update_pos(self, drone_pos, new_matrix, wp_path, drone_yaw, tf_prefix, dict_of_drones_pos):
        """This function is being called in DroneInjector, every time a pose is received for this drone.
            :param
                new_drone_pos - received from Father's subscriber.
                new_matrix - received from Father. same for all drones.
                wp_path - specific for this drone
                """
        self.matrix = new_matrix
        self.pos[0] = drone_pos[0][0]
        self.pos[1] = drone_pos[0][1]
        self.agent.wp_path = wp_path

        # # Perform rotation
        # if (rospy.Time.now().to_sec() - self.last_time_rot_called) >= self.rot_time_thresh:
        #     self.drone_yaw = drone_yaw + (np.pi / 2)
        #     self.drone_yaw = np.mod(self.drone_yaw, np.pi)
        #     self.last_time_rot_called = rospy.Time.now().to_sec()

        # Assume that new_pos = [x,y,z,r,p,y]
        # Find the path to the next wp using A* algorithm
        Astar_Movement = Astar.build_trj(drone_pos, self.env_limits, self.res, self.matrix, wp_path[0], tf_prefix, dict_of_drones_pos)
        # Update the path to goal in Agent
        self.agent.wp_path = Astar_Movement + self.agent.wp_path

        # Choose the relevant point from path according to current position
        self.agent.preform_step_sys_sim(drone_pos, self.drone_yaw, self.matrix)

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

    DronesManager:
        :param
            prefix_list - list of drone names
            num_of_drones
            limits - x_lim, y_lim - the size of matrice that will be extract from GRID
            resolution - tail size

        subscribes:
            grid - (we have an example in agent_0)
            position per drone - (we have an example in agent_0)

        contains:
            Local_planner's - per drone. an object that will publish the next point after inner filtering and
            postprocessing.
    """

    def __init__(self, prefix_takeoff_dict_input, env_limits_input, resolution, rate):
        # Hold all drones.
        self.cj_injector_container = []
        self.res = resolution
        # Number of drones.
        self.num_of_drones = len(prefix_takeoff_dict_input.keys())
        self.env_limits = env_limits_input
        self.min_dist_between_drones = 15  # tails
        self.matrix = np.zeros([np.int64(np.ceil((self.env_limits[1] - self.env_limits[0]) / self.res)),
                                np.int64(np.ceil((self.env_limits[3] - self.env_limits[2]) / self.res))])
        self.drones_pos_list = dict()
        self.rate = rate

        # Initiate DroneCjInjector per drone and add it to container.
        for drone_pref in prefix_takeoff_dict_input:
            rospy.loginfo("\n\n******************* Initiating DroneCjInjector for drone: {}".format(drone_pref))
            drone_init_takeoff = prefix_takeoff_dict_input[drone_pref]
            localplanner = Local_planner(drone_pref, drone_init_takeoff, self.env_limits, self.matrix, resolution)
            self.cj_injector_container.append(localplanner)
            self.drones_pos_list[drone_pref] = DronePosGoal(
                pos=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm],
                next_pos=[], goal=[drone_init_takeoff[0] * m_to_cm, drone_init_takeoff[1] * m_to_cm], yaw=[])

        # Init listeners:
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser)

    def grid_parser(self, msg):
        """ Every OccupancyGrid update, loop on all drones in container.

            Get theirs positions from tfBuffer.
            Give them new goal, if POI is not empty, update the drone new_pos in DroneManager.

        """
        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)
        self.matrix = np.array(msg.data).reshape((grid_width, grid_height))
        x_lim = self.env_limits[0:2]
        y_lim = self.env_limits[2:4]

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

            # todo: add here the paths for each drone from multi path planner
            wp_path = cur_pos[0]

            self.drones_pos_list[drone.tf_prefix].pos = cur_pos[0]
            self.drones_pos_list[drone.tf_prefix].goal = wp_path[0]

            self.drones_pos_list = drone.update_pos(cur_pos, self.matrix, wp_path, yaw, drone.tf_prefix, self.drones_pos_list)

if __name__ == '__main__':
    rospy.init_node("Local_planner", log_level=rospy.DEBUG)

    # Get params from ROS launch file.
    prefix_list_from_launch_file = rospy.get_param("~prefix_list")
    initial_takeoff_list_from_launch_file = rospy.get_param("~init_takeoff_list")
    limits_from_launch_file = rospy.get_param("~env_lim")
    res_from_launch_file = rospy.get_param("~resolution")
    nDrones = rospy.get_param("~nDrones")
    # localRate = rospy.get_param("~localRate") #10Hz
    localRate = 10

    x_lim = (limits_from_launch_file[0][0], limits_from_launch_file[0][1])
    y_lim = (limits_from_launch_file[0][2], limits_from_launch_file[0][3])

    limits_from_launch_file = [x_lim[0], x_lim[1], y_lim[0], y_lim[1]]

    prefix_takeoff_dict = dict()

    for iDrone in range(nDrones):
        pref = prefix_list_from_launch_file[iDrone]
        curr_takeoff = initial_takeoff_list_from_launch_file[iDrone]
        prefix_takeoff_dict[pref[0]] = curr_takeoff

    drone_container = DronesManager(prefix_takeoff_dict, limits_from_launch_file, res_from_launch_file, localRate)
    rospy.spin()
