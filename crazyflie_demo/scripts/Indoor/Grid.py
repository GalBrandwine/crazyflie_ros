#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from crazyflie_driver.msg import GenericLogData
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, MapMetaData
from shapely.geometry import Point, LineString, Polygon
from descartes import PolygonPatch
from bresenham import bresenham
import time
import threading
from geometry_msgs.msg import Pose
import tf2_ros
from tf.transformations import euler_from_quaternion


m_to_cm = 100

# Drone position
class drone_pos:
    def __init__(self, time = None, x = None, y = None, z = None, w = None, index = -1):

        self.time = time
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.index = index


# Drone point cloud (from range sensors)
class drone_pc:
    def __init__(self, time=None, pc_sens=None):
        self.time = time
        self.pc_sens = pc_sens


class Grid:

    def __init__(self, border_polygon, res, nDrones, initial_pos_dict):

        self.x_lim = [border_polygon[0][0], border_polygon[0][0]]
        self.y_lim = [border_polygon[0][1], border_polygon[0][1]]

        for i in range(1, border_polygon.__len__()):
            if self.x_lim[0] > border_polygon[i][0]:
                self.x_lim[0] = border_polygon[i][0]
            if self.x_lim[1] < border_polygon[i][0]:
                self.x_lim[1] = border_polygon[i][0]
            if self.y_lim[0] > border_polygon[i][1]:
                self.y_lim[0] = border_polygon[i][1]
            if self.y_lim[1] < border_polygon[i][1]:
                self.y_lim[1] = border_polygon[i][1]

        self.res = res
        self.matrix = np.zeros([np.int64(np.ceil((self.x_lim[1]-self.x_lim[0])/self.res)), np.int64(np.ceil((self.y_lim[1]-self.y_lim[0])/self.res))])
        self.nDrones = nDrones
        self.eps = 1  # Allowed time [sec] difference between messages
        self.drones_pos_list = dict()
        self.drones_pc_list = dict()
        # maximal limit for pc delta from drone reference in cm
        self.takeofpos = initial_pos_dict
        self.initpos = [0, 0, 0]  # Reference point
        self.topics_arr = []
        for iDrone in range(self.nDrones):
            drone_name = rospy.get_param("~drone_name_{}".format(iDrone))
            self.topics_arr.append("/{}/point_cloud".format(drone_name))
        self.drone_name_arr = []
        self.floor_thr = 32
        self.sens_limit = 100

        for i, id in enumerate(initial_pos_dict):
            self.drones_pos_list[id] = drone_pos(time=0, x=initial_pos_dict[id][0],
                                                 y=initial_pos_dict[id][1],
                                                 z=initial_pos_dict[id][2], w=None, index=i)

        for iDrone in range(self.nDrones):
            # Init listeners
            drone_name = rospy.get_param("~drone_name_{}".format(iDrone))
            self.drone_name_arr.append(drone_name)
            # self.pos_sub = rospy.Subscriber("/{}/log_pos".format(drone_name), GenericLogData,
            #                                 callback = self.pos_parser)
            self.pc_sub = rospy.Subscriber("/{}/point_cloud".format(drone_name), PointCloud2,
                                           callback = self.point_cloud_parser, callback_args = "/{}/point_cloud".format(drone_name))

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Start occupancy grid publisher
        grid_pub_thread = threading.Thread(name='grid_pub_thread', target=self.init_grid_publisher)
        grid_pub_thread.start()

    def point_cloud_parser(self, msg, topic):
        """Each publicitation, theres' an array of 10 points."""
        point_cloud_last_timestamp = msg.header
        point_cloud = pc2.read_points_list(msg, skip_nans=True)

        sens = []
        drone_id = []
        plt_index = []
        for j in range(len(self.topics_arr)):
            cur_topic = self.topics_arr[j]
            if topic == cur_topic:
                cur_topic_list = cur_topic.split("/")
                drone_id = [s for s in cur_topic_list if "cf" in s][0]
                plt_index = self.drones_pos_list[drone_id].index


        try:
            trans = self.tfBuffer.lookup_transform('world', drone_id, rospy.Time(0))

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

            self.pos = [x, y, z, roll, pitch, yaw]
            # rospy.loginfo("pos in Grid: {}\n".format(self.pos))

            # Store drone position and convert it from [m] to [cm]
            self.drones_pos_list[drone_id] = drone_pos(point_cloud_last_timestamp.stamp.secs, x*m_to_cm, y*m_to_cm, z*m_to_cm, yaw, plt_index)

            # Change tail to be empty if the drone is in that tail.
            i, j = self.xy_to_ij(self.drones_pos_list[drone_id].x, self.drones_pos_list[drone_id].y)
            if self.matrix[i][j] == 0:
                self.change_tail_to_empty(i, j)
                self.empty_idxs.append([i, j])

        except:
            rospy.logdebug("tf lookup -- {} not found".format(drone_id))
            # print("after PC gets except")

        # Read data from all sensors (probably 4)
        for i in range(len(point_cloud)):
            point = point_cloud[i]
            # rospy.loginfo([point.z*m_to_cm])
            if point.z*m_to_cm > self.floor_thr:
                sens.append([point.x*m_to_cm, point.y*m_to_cm, point.z*m_to_cm])
            if sens and drone_id:
                self.drones_pc_list[drone_id] = drone_pc(point_cloud_last_timestamp.stamp.secs, sens)
                # Update grid using the new data
                self.update_from_tof_sensing_list(drone_id)
                self.complete_wall_in_corners(self.matrix)


    # def pos_parser(self, msg):
    #     pos_header = msg.header
    #     pos_val = msg.values
    #
    #     drone_id = pos_header.frame_id.split("/")[0] # Extract drone name from topic name
    #     plt_index = self.drones_pos_list[drone_id].index
    #     # Store drone position and convert it from [m] to [cm]
    #     self.drones_pos_list[drone_id] = drone_pos(pos_header.stamp.secs,
    #                                                self.takeofpos[drone_id][0]+(pos_val[0]*m_to_cm),
    #                                                self.takeofpos[drone_id][1]+(pos_val[1]*m_to_cm),
    #                                                self.takeofpos[drone_id][2]+(pos_val[2]*m_to_cm), None, plt_index)
    #
    #     # Change tail to be empty if the drone is in that tail.
    #     i, j = self.xy_to_ij(self.drones_pos_list[drone_id].x, self.drones_pos_list[drone_id].y)
    #     if self.matrix[i][j] == 0:
    #         self.change_tail_to_empty(i, j)


    # Initialize a publisher for occupancy grid
    def init_grid_publisher(self):
        self.grid_publisher = rospy.Publisher('/indoor/occupancy_grid_topic', OccupancyGrid, queue_size=10)
        # OccupancyGrid documentation:
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html
        occ_grid_msg = OccupancyGrid()

        rate = rospy.Rate(2)  # 2 Hz
        while not rospy.is_shutdown():
            m = MapMetaData() # Grid metadata
            m.resolution = self.res # Grid resolution
            m.width = self.x_lim[1] - self.x_lim[0] # Grid width in world CS
            m.height = self.y_lim[1] - self.y_lim[0] # Grid height in worlds CS
            m.origin = Pose() # The grid origin in world CS (there is no need for indoor navigation)
            occ_grid_msg.info = m

            occ_grid_msg.header.stamp = rospy.Time.now()
            occ_grid_msg.header.frame_id = "/indoor/occupancy_grid"

            # Convert the matrix from 2D fload64 to 1D int8 list
            occ_grid_msg.data = list(np.asarray(self.matrix.flatten(), dtype=np.int8))

            # Publish the message
            self.grid_publisher.publish(occ_grid_msg)
            rate.sleep()

    # Check if two time stamps are close enough (self.eps is the threshold)
    def is_time_equal(self, t1, t2):
        return abs(t1-t2) <= self.eps

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def change_tail_to_empty(self, i, j):
        self.matrix[i][j] = 1

    def change_tail_to_wall(self, i, j):
        self.matrix[i][j] = 2
        # self.matrix[i][j] += 1

    def update_from_tof_sensing_list(self, drone_id):
        current_pos = self.drones_pos_list[drone_id]
        current_pc = self.drones_pc_list[drone_id]

        # Check if the current point cloud was taken close enough to last position message time
        # (i.e. was taken from the current position of the drone).
        if not self.is_time_equal(current_pos.time, current_pc.time):
            return

        for elem in current_pc.pc_sens:
            sensing_pos = [[self.initpos[0]+elem[0], self.initpos[1]+elem[1]]]
            self.update_with_tof_sensor([[current_pos.x, current_pos.y]], sensing_pos)

    def update_with_tof_sensor(self, sensor_pos, tof_sensing_pos):
        i0, j0 = self.xy_to_ij(sensor_pos[0][0], sensor_pos[0][1])
        i1, j1 = self.xy_to_ij(tof_sensing_pos[0][0], tof_sensing_pos[0][1])
        bres_list = list(bresenham(i0, j0, i1, j1))
        bres_list = bres_list[:-1]
        # num_of_samples = int(np.floor(np.linalg.norm(np.subtract(tof_sensing_pos, sensor_pos)) / self.res * 1))
        # xs = np.linspace(sensor_pos[0][0], tof_sensing_pos[0][0], num=num_of_samples, endpoint=True)
        # ys = np.linspace(sensor_pos[0][1], tof_sensing_pos[0][1], num=num_of_samples, endpoint=True)
        for ind in range(len(bres_list)):
        # for ind in range(1, num_of_samples):
            # i, j = self.xy_to_ij(xs[ind], ys[ind])
            i, j = bres_list[ind]
            if 0 > i or i >= self.matrix.shape[0] or 0 > j or j >= self.matrix.shape[1]:
                return
            if self.matrix[i][j] == 0 and np.linalg.norm(np.subtract([i, j], [i0, j0])) < (self.sens_limit / self.res):
            # if self.matrix[i][j] == 0 and np.linalg.norm(np.subtract([xs[ind], ys[ind]], sensor_pos)) < (self.sens_limit):
                self.change_tail_to_empty(i, j)
        d = np.subtract(tof_sensing_pos, sensor_pos)
        norm_d = np.linalg.norm(d)
        if norm_d > 0 and np.linalg.norm(np.subtract([i1, j1], [i0, j0])) < (self.sens_limit / self.res):
        # if norm_d > 0 and np.linalg.norm(np.subtract(tof_sensing_pos, sensor_pos)) < (self.sens_limit):
            wall_pos = tof_sensing_pos + d / norm_d * self.res / 1000
            i, j = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
            self.change_tail_to_wall(i, j)


    def complete_wall_in_corners(self, matrix):
        for i in range(1, matrix.__len__()-1):
            for j in range(1, matrix[i].__len__()-1):
                if matrix[i][j] == 0:
                    if ((matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1 )) or
                        (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1 )) or
                        (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1 )) or
                        (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1 ))):
                        self.change_tail_to_wall(i, j) # Originally
        j = 0
        for i in range(1, matrix.__len__()-1):
            if (matrix[i][j] == 0 and
                    (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1)) or
                    (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1))):
                self.change_tail_to_wall(i, j)

        j = matrix[0].__len__()-1
        for i in range(1, matrix.__len__() - 1):
            if (matrix[i][j] == 0 and
                    (matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1)) or
                    (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1))):
                self.change_tail_to_wall(i, j)

        i = 0
        for j in range(1, matrix[0].__len__()-1):
            if (matrix[i][j] == 0 and
                    (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1)) or
                    (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1))):
                self.change_tail_to_wall(i, j)

        i = matrix.__len__()-1
        for j in range(1, matrix[0].__len__() - 1):
            if (matrix[i][j] == 0 and
                    (matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1)) or
                    (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1))):
                self.change_tail_to_wall(i, j)

        if (matrix[0][0] == 0 and
                    (matrix[0][1] == 2 and matrix[1][0] == 2)):
            self.change_tail_to_wall(0, 0)


        if (matrix[0][matrix[0].__len__()-1] == 0 and
                (matrix[0][matrix[0].__len__()-2] == 2 and matrix[1][matrix[0].__len__()-1] == 2)):
            self.change_tail_to_wall(0, matrix[0].__len__()-1)


        if (matrix[matrix.__len__()-1][matrix[0].__len__()-1] == 0 and
                (matrix[matrix[0].__len__()-1][matrix[0].__len__()-2] == 2 and matrix[matrix[0].__len__()-2][matrix[0].__len__()-1] == 2)):
            self.change_tail_to_wall(matrix.__len__()-1, matrix[0].__len__()-1)


        if (matrix[matrix.__len__()-1][0] == 0 and
                (matrix[matrix[0].__len__()-1][1] == 2 and matrix[matrix[0].__len__()-2][0] == 2)):
            self.change_tail_to_wall(matrix.__len__()-1, 0)

if __name__ == "__main__":

    rospy.init_node("grid_builder")

    # grid = Grid([(0, 0), (570, 0), (570, 550), (0, 550), (0, 0)], 10)
    env_lim = rospy.get_param("~env_lim")
    env_space = rospy.get_param("~env_space")
    resolution = rospy.get_param("~resolution")
    nDrones = rospy.get_param("~nDrones")

    exec("env_lim = {}".format(env_lim))

    x_lim = (env_lim[0] - env_space, env_lim[1] + env_space)
    y_lim = (env_lim[2] - env_space, env_lim[3] + env_space)

    polygon_border = [(x_lim[0], y_lim[0]), (x_lim[1], y_lim[0]), (x_lim[1], y_lim[1]),
                      (x_lim[0], y_lim[1]), (x_lim[0], y_lim[0])]

    initial_pos_dict = dict()
    for iDrone in range(nDrones):
        curr_drone_name = rospy.get_param("~drone_name_{}".format(iDrone))
        curr_drone_takeoff_pos = rospy.get_param("~drone_takeoff_position_{}".format(iDrone))
        exec ("curr_drone_takeoff_pos = {}".format(curr_drone_takeoff_pos))
        initial_pos_dict[curr_drone_name] = curr_drone_takeoff_pos

    grid = Grid(polygon_border, resolution, nDrones, initial_pos_dict)
