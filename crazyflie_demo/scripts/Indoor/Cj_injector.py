#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
import numpy as np
from math import radians
from threading import Thread

import rospy
import tf
import tf2_ros
from GridPOI import GridPOI
from Agent import Agent
import PathBuilder
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


def to_pose_stamped(x, y, z, roll, pitch, yaw):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose

def is_los(p1, p2, matrix, x_lim, y_lim, res):
    n = int(np.maximum(1, np.ceil(np.linalg.norm(p1-p2)/res)*3))
    x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
    y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
    for ind in range(1, n):
        i, j = xy_to_ij(x[ind], y[ind], x_lim, y_lim, res)
        if matrix[i][j] != 1:
            return False
    return True


def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0])/res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j


def get_goal_point(pos, temp_interesting_points_list_xy, matrix, x_lim, y_lim, res):
    g_idx = 0
    dist_arr = []
    for idx, elem in enumerate(temp_interesting_points_list_xy):
        dist_arr.append(np.linalg.norm(np.subtract(elem, pos[0])))
    sorted_dist_idxs = sorted(range(len(dist_arr)), key=lambda k: dist_arr[k])
    for idx in sorted_dist_idxs:
        if is_los(pos, [[temp_interesting_points_list_xy[idx][0], temp_interesting_points_list_xy[idx][1]]], matrix, x_lim, y_lim, res):
            g_idx = idx
            break
    return g_idx


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
        x = drone_init_takeoff_input[0]
        y = drone_init_takeoff_input[1]
        z = drone_init_takeoff_input[2]
        self.pos = [x, y, z, 0, 0, 0]
        self.next_pose = self.pos
        self.agent = Agent(self.tf_prefix, [self.pos[0:2]], self.res, self.env_limits)
        self.pathfinder = None
        self.matrix = matrix
        # Init publisher
        self.Cj_injector_pub = rospy.Publisher('/' + self.tf_prefix + "/Cj_injcetor", PoseStamped,
                                               queue_size=1)  # hover message publisher

    def update_pos(self, drone_pos, new_matrix, next_point, corrners_array):
        """This function is being called in DroneInjector, every time a pose is recieved for this drone.
            :param
                new_drone_pos - received from Father's subscriber.
                new_matrix - recieved fromFather. same for all drones.
                nex_point - specific for this drone, calculated in Father's POI function
                corrners_array - allowed land marks,
                """
        self.matrix = new_matrix
        self.pos = drone_pos
        # Assume that new_pos = [x,y,z,r,p,y]
        Astar_Movement = PathBuilder.build_trj([self.pos[0:2]], self.env_limits, self.res, self.matrix, corrners_array,
                                               next_point)
        self.agent.astar_path = Astar_Movement[0]
        temp_yaw = np.random.rand() * np.pi / 4 # todo: A function which computing the yaw angle should be placed here
        self.agent.preform_step_sys_sim([self.pos[0:2]], temp_yaw, self.matrix)
        # self.next_pose[0:2] = self.agent.next_pos

        self.next_pose[0:2] = next_point
        self.next_pose[2:7] = self.pos[2:7]
        self.next_pose[2] = 0.35 # hard coded Z height

        x = self.next_pose[0]
        y = self.next_pose[1]
        z = self.next_pose[2]
        roll = 0
        pitch = 0
        yaw = self.next_pose[3]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # rospy.logdebug("cj_injection injected this pose: \n{}".format(pose))
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
        self.POI = GridPOI(resolution, env_limits_input)
        self.matrix = np.zeros([np.int64(np.ceil((self.env_limits[1] - self.env_limits[0]) / self.res)),
                                np.int64(np.ceil((self.env_limits[3] - self.env_limits[2]) / self.res))])
        # initiate DroneCjInjector per drone and add it to container
        for drone in prefix_takeoff_dict_input:
            rospy.logdebug("\n\n******************* Initiating DroneCjInjector for drone: {}".format(drone))

            drone_init_takeoff = prefix_takeoff_dict_input[drone]
            drone = DroneCjInjector(drone, drone_init_takeoff, self.env_limits, self.matrix, resolution)
            self.cj_injector_container.append(drone)

        self.r = rate

        # Init listeners: Fill here when ready:

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser)
        # self.rotation_example_subscriber = rospy.Subscriber("/Cj_injection_rotation_example", Empty, self.launcher) # Why is it here?

    def grid_parser(self, msg):

        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)

        # self.matrix = np.array(msg.data).reshape((grid_height, grid_width))
        # if rospy.time.now() - self.last_time_POI_called >= self.thresh:
        #     self.last_time_POI_called = rospy.time.now()
        #     [_, interesting_points_list_xy, _, corner_points_list_xy] = self.POI.find_POI(self.matrix)
        [_, interesting_points_list_xy, _, corner_points_list_xy] = self.POI.find_POI(self.matrix)
        x_lim = self.env_limits[0:2]
        y_lim = self.env_limits[2:4]

        # corner_points_list_xy = [] # Temporary! only for debug

        for drone in self.cj_injector_container:

            try:
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

                pos = [x, y, z, roll, pitch, yaw]
                # rospy.loginfo("pos in Display: {}\n".format(self.pos))
                cur_pos = [[pos[0], pos[1]]]

                g_idx = get_goal_point(cur_pos, interesting_points_list_xy, self.matrix, x_lim, y_lim, self.res)
                gx = interesting_points_list_xy[g_idx][0]
                gy = interesting_points_list_xy[g_idx][1]

                # drone.update_pos(pos, self.matrix, pos[0:2], corner_points_list_xy)
                drone.update_pos(pos, self.matrix, [gx, gy], corner_points_list_xy)
                del interesting_points_list_xy[g_idx]

                # rospy.logdebug("in grid_parser - drone.update:\n"
                #                "pos: {}\n"
                #                "next_pos: {}\n"
                #                "".format(pos,pos[0:2]))

            except:
                rospy.logdebug("tf lookup -- {} not found".format(drone.tf_prefix))
            # except Exception as e:
            #     rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node("incjetor", log_level=rospy.DEBUG)

    prefix_list_from_launch_file = rospy.get_param("~prefix_list")
    initial_takeoff_list_from_launch_file = rospy.get_param("~init_takeoff_list")
    limits_from_launch_file = rospy.get_param("~env_lim")
    res_from_launch_file = rospy.get_param("~resolution")
    env_space = rospy.get_param("~env_space")
    nDrones = rospy.get_param("~nDrones")

    # exec ("limits_from_launch_file = {}".format(limits_from_launch_file))
    # exec ("initial_takeoff_list_from_launch_file = {}".format(initial_takeoff_list_from_launch_file))
    rospy.logdebug(limits_from_launch_file)

    x_lim = (limits_from_launch_file[0][0] - env_space, limits_from_launch_file[0][1] + env_space)
    y_lim = (limits_from_launch_file[0][2] - env_space, limits_from_launch_file[0][3] + env_space)

    limits_from_launch_file = [x_lim[0], x_lim[1], y_lim[0], y_lim[1]]


    rospy.logdebug(prefix_list_from_launch_file)
    rospy.logdebug(initial_takeoff_list_from_launch_file)

    prefix_takeoff_dict = dict()
    for iDrone in range(nDrones):

        pref = prefix_list_from_launch_file[iDrone]
        curr_takeoff = initial_takeoff_list_from_launch_file[iDrone]
        rospy.logdebug(pref)
        rospy.logdebug(curr_takeoff[1])
        prefix_takeoff_dict[pref] = curr_takeoff

    drone_container = DroneInjector(prefix_takeoff_dict, limits_from_launch_file, res_from_launch_file, 15)

    rospy.loginfo("******************* Publish /Cj_injection_rotation_example for starting the example")

    rospy.spin()
