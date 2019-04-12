#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from crazyflie_driver.msg import GenericLogData
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

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
        self.eps = 1 # allowed time difference between messages
        self.m_to_cm = 100
        self.min_rng = 20

        # For debug only:
        self.pos_scatter = None # A handle to scatter plot of drone position

        self.drones_pos_list = dict()
        self.drones_pc_list = dict()
        for iDrone in range(self.nDrones):
            # Init listeners
            self.pc_sub = rospy.Subscriber("/cf4/point_cloud", PointCloud2,
                                           self.point_cloud_parser)
            self.pos_sub = rospy.Subscriber("/cf4/log_pos", GenericLogData,
                                            self.pos_parser)

        # Start occupancy grid piblisher
        self.init_grid_publisher()



    def point_cloud_parser(self, msg):
        """Each publicitation, theres' an array of 10 points."""
        point_cloud_last_timestamp = msg.header
        point_cloud = pc2.read_points_list(msg, skip_nans=True)

        sens = []
        # Read data from all sensors (probably 4)
        for i in range(len(point_cloud)):
            point = point_cloud[i]
            sens.append([point.x*self.m_to_cm, point.y*self.m_to_cm, point.z*self.m_to_cm])

        drone_id = point_cloud_last_timestamp.frame_id
        self.drones_pc_list[drone_id] = drone_pc(point_cloud_last_timestamp.stamp.secs, sens)

        # Update grid using the new data
        self.update_from_tof_sensing_list(drone_id)


    def pos_parser(self, msg):
        pos_header = msg.header
        pos_val = msg.values
        # rospy.loginfo(self.pos_header)
        # rospy.loginfo(self.pos_val)

        drone_id = pos_header.frame_id.split("/")[0] # Extract drone name from topic name
        # Store drone position and convert it from [m] to [cm]
        self.drones_pos_list[drone_id] = drone_pos(pos_header.stamp.secs, pos_val[0]*self.m_to_cm, pos_val[1]*self.m_to_cm, pos_val[2]*self.m_to_cm, None)

        # For debug only:
        if self.pos_scatter is not None:
            self.pos_scatter.remove() # Remove previous position scatter plot
        self.pos_scatter = plt.scatter(self.drones_pos_list[drone_id].x, self.drones_pos_list[drone_id].y, s=100, c='r')
        plt.draw()
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        plt.pause(0.0000000001)

    # Initialize a publisher for occupancy grid
    def init_grid_publisher(self):
        self.grid_publisher = rospy.Publisher('/indoor/occupancy_grid_topic', OccupancyGrid, queue_size=10)
        # OccupancyGrid documentation:
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html
        occ_grid_msg = OccupancyGrid()
        rate = rospy.Rate(0.5)  # 0.5 Hz
        while not rospy.is_shutdown():
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
        # self.change_tail_color_ij(i, j, 'k')
        self.matrix[i][j] = 1

    def change_tail_to_wall(self, i, j):
        # self.change_tail_color_ij(i, j, 'w')
        self.matrix[i][j] = 2

    def update_from_tof_sensing_list(self, drone_id):
        current_pos = self.drones_pos_list[drone_id]
        current_pc = self.drones_pc_list[drone_id]
        if not self.is_time_equal(current_pos.time, current_pc.time):
            return

        self.empty_idxs = []
        self.wall_idxs = []

        for elem in current_pc.pc_sens:
            if np.linalg.norm([elem[0], elem[1]]) >= self.min_rng:
                sensing_pos = [[current_pos.x+elem[0], current_pos.y+elem[1]]]
                self.update_with_tof_sensor([[current_pos.x, current_pos.y]], sensing_pos, 1)

    def update_with_tof_sensor(self, sensor_pos, tof_sensing_pos, is_tof_senses):
        num_of_samples = int(np.floor(np.linalg.norm(np.subtract(tof_sensing_pos, sensor_pos)) / self.res * 2))
        xs = np.linspace(sensor_pos[0][0], tof_sensing_pos[0][0], num=num_of_samples, endpoint=True)
        ys = np.linspace(sensor_pos[0][1], tof_sensing_pos[0][1], num=num_of_samples, endpoint=True)
        for ind in range(1, num_of_samples):
            i, j = self.xy_to_ij(xs[ind], ys[ind])
            if 0 > i or i >= self.matrix.shape[0] or 0 > j or j >= self.matrix.shape[1]:
                return
            if self.matrix[i][j] == 0:
                self.change_tail_to_empty(i, j)
                self.empty_idxs.append([i,j])
        if is_tof_senses:
            d = tof_sensing_pos - sensor_pos
            norm_d = np.linalg.norm(d)
            if norm_d > 0:
                wall_pos = tof_sensing_pos + d /norm_d*self.res/1000
                i, j = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
                self.change_tail_to_wall(i, j)
                self.wall_idxs.append([i, j])


if __name__ == "__main__":

    rospy.init_node("grid_builder")

    grid = Grid([(0, 0), (570, 0), (570, 550), (0, 550), (0, 0)], 10)

    # For debug only:
    plt.ion()
    plt.show()

    rospy.spin()