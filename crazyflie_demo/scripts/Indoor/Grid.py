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


# Drone position
class drone_pos:
    def __init__(self, time = None, x = None, y = None, z = None, w = None):

        self.time = time
        self.x = x
        self.y = y
        self.z = z
        self.w = w


# Drone point cloud (from range sensors)
class drone_pc:
    def __init__(self, time=None, pc_sens=None):
        self.time = time
        self.pc_sens = pc_sens


class Grid:

    def __init__(self, border_polygon, res):
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
        self.empty_idxs = []
        self.wall_idxs = []
        self.nDrones = 1
        self.eps = 1 # Allowed time difference between messages
        self.m_to_cm = 100
        self.min_rng = 20 # Range which indicates equal positions
        self.drones_pos_list = dict()
        self.drones_pc_list = dict()
        # maximal limit for pc delta from drone reference in cm
        self.pc_lim = 200

        ###################################### Only for debug ##################################################
        self.initpos = [0, 0, 0]  # Reference point
        self.takeofpos = [100, 100, 0] # Take off position
        self.tail_handles = list()
        self.drone_handel = []
        self.fig = plt.figure()
        ax = self.fig.subplots(1, 1)
        plt.axis([self.x_lim[0], self.x_lim[1], self.y_lim[0], self.y_lim[1]])
        self.ax = ax
        for i in range(0, self.matrix.__len__()):
            handles_list = list()
            for j in range(0, self.matrix[i].__len__()):
                handles_list.append(self.plot_ij(i, j))
            self.tail_handles.append(handles_list)

        self.drone_handel, = self.ax.plot(self.takeofpos[0], self.takeofpos[1], 'ob')

        self.fig.show()
        self.fig.canvas.draw()
        plt.pause(0.001)
        ########################################################################################################

        for iDrone in range(self.nDrones):
            # Init listeners
            self.pos_sub = rospy.Subscriber("/cf6/log_pos", GenericLogData,
                                            self.pos_parser)
            self.pc_sub = rospy.Subscriber("/cf6/point_cloud", PointCloud2,
                                           self.point_cloud_parser)


        # Start occupancy grid publisher
        grid_pub_thread = threading.Thread(name='grid_pub_thread', target=self.init_grid_publisher)
        grid_pub_thread.start()

    ###################################### Only for debug ##################################################
    def plot_ij(self, i, j):
        pol_center = self.ij_to_xy(i, j)
        tail = Polygon([(pol_center[0] - self.res / 2, pol_center[1] - self.res / 2),
                        (pol_center[0] - self.res / 2, pol_center[1] + self.res / 2)
                           , (pol_center[0] + self.res / 2, pol_center[1] + self.res / 2),
                        (pol_center[0] + self.res / 2, pol_center[1] - self.res / 2)
                           , (pol_center[0] - self.res / 2, pol_center[1] - self.res / 2)])
        return self.ax.add_patch(PolygonPatch(tail, facecolor='gray'))

    def change_tail_color_ij(self, i, j, color):
        self.tail_handles[i][j].set_fc(color)
        ########################################################################################################


    def point_cloud_parser(self, msg):
        """Each publicitation, theres' an array of 10 points."""
        point_cloud_last_timestamp = msg.header
        point_cloud = pc2.read_points_list(msg, skip_nans=True)

        sens = []
        # Read data from all sensors (probably 4)
        for i in range(len(point_cloud)):
            point = point_cloud[i]
            sens.append([point.x*self.m_to_cm, point.y*self.m_to_cm, point.z*self.m_to_cm])
            # drone_id = point_cloud_last_timestamp.frame_id # need to change frame_id from "world" to cf6
            drone_id = "cf6"
            # rospy.loginfo("PC")
            # rospy.loginfo(drone_id)
            self.drones_pc_list[drone_id] = drone_pc(point_cloud_last_timestamp.stamp.secs, sens)
            # Update grid using the new data
            self.update_from_tof_sensing_list(drone_id)


    def pos_parser(self, msg):
        pos_header = msg.header
        pos_val = msg.values

        drone_id = pos_header.frame_id.split("/")[0] # Extract drone name from topic name
        # Store drone position and convert it from [m] to [cm]
        self.drones_pos_list[drone_id] = drone_pos(pos_header.stamp.secs, self.takeofpos[0]+(pos_val[0]*self.m_to_cm), self.takeofpos[1]+(pos_val[1]*self.m_to_cm), self.takeofpos[2]+(pos_val[2]*self.m_to_cm), None)
        # rospy.loginfo("Pos")
        # rospy.loginfo(drone_id)
        i, j = self.xy_to_ij(self.drones_pos_list[drone_id].x, self.drones_pos_list[drone_id].y)
        if self.matrix[i][j] == 0:
            self.change_tail_to_empty(i, j)
            self.empty_idxs.append([i, j])


    # Initialize a publisher for occupancy grid
    def init_grid_publisher(self):
        self.grid_publisher = rospy.Publisher('/indoor/occupancy_grid_topic', OccupancyGrid, queue_size=10)
        # OccupancyGrid documentation:
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html
        occ_grid_msg = OccupancyGrid()

        rate = rospy.Rate(0.5)  # 0.5 Hz
        while not rospy.is_shutdown():
            m = MapMetaData() # Grid metadata
            m.resolution = self.res # Grid resolution
            m.width = self.x_lim[1] - self.x_lim[0] # Grid width in world CS
            m.height = self.y_lim[1] - self.y_lim[0] # Grid height in worlds CS
            m.origin = Pose() # The grid origin in world CS (there is no need for indoor navigation)
            occ_grid_msg.info = m

            occ_grid_msg.header.stamp = rospy.Time.now()
            occ_grid_msg.header.frame_id = "/indoor/occupancy_grid"

            # Convert the matrix from 2D fload64 to 1D uin8 list
            occ_grid_msg.data = list(np.asarray(self.matrix.flatten(), dtype=np.uint8))

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
        self.change_tail_color_ij(i, j, 'k')
        self.matrix[i][j] = 1

    def change_tail_to_wall(self, i, j):
        self.change_tail_color_ij(i, j, 'w')
        self.matrix[i][j] = 2

    def update_from_tof_sensing_list(self, drone_id):
        current_pos = self.drones_pos_list[drone_id]
        current_pc = self.drones_pc_list[drone_id]
        if not self.is_time_equal(current_pos.time, current_pc.time):
            return

        self.empty_idxs = []
        self.wall_idxs = []

        for elem in current_pc.pc_sens:
            if abs(elem[0]) < self.pc_lim and abs(elem[1]) < self.pc_lim and np.linalg.norm(elem) > 0:
                sensing_pos = [[self.initpos[0]+elem[0], self.initpos[1]+elem[1]]]
                self.update_with_tof_sensor([[current_pos.x, current_pos.y]], sensing_pos)

                ###################################### Only for debug ##################################################
                self.drone_handel.set_data(current_pos.x, current_pos.y)
                self.fig.canvas.draw()

                ########################################################################################################

    # def update_with_tof_sensor(self, sensor_pos, tof_sensing_pos):
    #     si, sj = self.xy_to_ij(sensor_pos[0][0], sensor_pos[0][1])
    #     d = np.subtract(tof_sensing_pos, sensor_pos)
    #     norm_d = np.linalg.norm(d)
    #     wall_pos = tof_sensing_pos + d / norm_d * self.res / 1000
    #     gi, gj = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
    #     bpath = list(bresenham(si, sj, gi, gj))
    #     for ii, elem in enumerate(bpath):
    #         if 0 > elem[0] or elem[0] >= self.matrix.shape[0] or 0 > elem[1] or elem[1] >= self.matrix.shape[1]:
    #             return
    #         if self.matrix[elem[0]][elem[1]] == 0:
    #             self.change_tail_to_empty(elem[0], elem[1])
    #             self.empty_idxs.append([elem[0], elem[1]])
    #     self.change_tail_to_wall(gi, gj)
    #     self.wall_idxs.append([gi, gj])


    def update_with_tof_sensor(self, sensor_pos, tof_sensing_pos):
        num_of_samples = int(np.floor(np.linalg.norm(np.subtract(tof_sensing_pos, sensor_pos)) / self.res * 3))
        xs = np.linspace(sensor_pos[0][0], tof_sensing_pos[0][0], num=num_of_samples, endpoint=True)
        ys = np.linspace(sensor_pos[0][1], tof_sensing_pos[0][1], num=num_of_samples, endpoint=True)
        for ind in range(1, num_of_samples):
            i, j = self.xy_to_ij(xs[ind], ys[ind])
            if 0 > i or i >= self.matrix.shape[0] or 0 > j or j >= self.matrix.shape[1]:
                return
            if self.matrix[i][j] == 0:
                self.change_tail_to_empty(i, j)
                self.empty_idxs.append([i, j])
        d = np.subtract(tof_sensing_pos, sensor_pos)
        norm_d = np.linalg.norm(d)
        if norm_d > 0:
            wall_pos = tof_sensing_pos + d / norm_d * self.res / 1000
            i, j = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
            self.change_tail_to_wall(i, j)
            self.wall_idxs.append([i, j])


if __name__ == "__main__":

    rospy.init_node("grid_builder")

    # grid = Grid([(0, 0), (570, 0), (570, 550), (0, 550), (0, 0)], 10)
    grid = Grid([(0, 0), (200, 0), (200, 200), (0, 200), (0, 0)], 10)

    plt.show(block=True)