#!/usr/bin/env python
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
from descartes import PolygonPatch
import numpy as np
import matplotlib.animation as manimation
import rospy
from crazyflie_driver.msg import GenericLogData
from Grid import drone_pos, m_to_cm
from nav_msgs.msg import OccupancyGrid

class Display_manager:
    def __init__(self, border_polygon, obs_array, x_lim, y_lim, res, matrix, initial_pos_dict, nDrones):

        self.nDrones = nDrones

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.matrix = matrix
        self.last_matrix = matrix
        self.border_polygon = border_polygon
        self.obs_array = obs_array

        self.fig = plt.figure()
        mgr = plt.get_current_fig_manager()
        # mgr.full_screen_toggle()

        ax_env, ax_grid = self.fig.subplots(1, 2)
        plt.axis([self.x_lim[0], self.x_lim[1], self.y_lim[0], self.y_lim[1]])

        self.ax_env = ax_env
        self.ax_grid = ax_grid

        self.tail_handles = list()
        self.corner_plot_handle = list()
        self.interest_plot_handle = list()

        self.plot_grid()
        self.plot_obs_array()

        self.takeofpos = initial_pos_dict
        init_pos = []
        self.drones_pos_list = dict()

        for i, id in enumerate(initial_pos_dict):
            self.drones_pos_list[id] = drone_pos(time = 0, x = initial_pos_dict[id][0],
                                                 y = initial_pos_dict[id][1],
                                                 z = initial_pos_dict[id][2], w = None, index = i)
            init_pos.append([self.drones_pos_list[id].x, self.drones_pos_list[id].y])

        self.plot_color = 'ob'

        self.plot_handle_envs = []
        self.plot_handle_grids = []
        # self.plot_handle_envs_vt = []
        # self.plot_handle_grids_vt = []
        # self.plot_handle_envs_l2vt = []
        # self.plot_handle_grids_l2vt = []
        # self.edg_to_neighbors_plot_handles = []
        # self.prev_interesting_points = []
        # self.prev_corner_points = []

        for pos in init_pos:
            plot_handle_env, = ax_env.plot(pos[0], pos[1], 'ob')
            self.plot_handle_envs.append(plot_handle_env, )
            plot_handle_grid, = ax_grid.plot(pos[0], pos[1], 'ob')
            self.plot_handle_grids.append(plot_handle_grid, )
            # plot_handle_env_vt, = ax_env.plot(pos[0], pos[1], 'or')
            # self.plot_handle_envs_vt.append(plot_handle_env_vt, )
            # plot_handle_grid_vt, = ax_grid.plot(pos[0], pos[1], 'or')
            # self.plot_handle_grids_vt.append(plot_handle_grid_vt, )
            # plot_handle_env_l2vt, = ax_env.plot(list(np.repeat(pos[0], 2)), list(np.repeat(pos[1], 2)), '--r')
            # self.plot_handle_envs_l2vt.append(plot_handle_env_l2vt, )
            # plot_handle_grid_l2vt, = ax_grid.plot(list(np.repeat(pos[0], 2)), list(np.repeat(pos[1], 2)), '--r')
            # self.plot_handle_grids_l2vt.append(plot_handle_grid_l2vt, )

            # self.edg_to_neighbors_plot_handles.append([])

        for iDrone in range(self.nDrones):
            # Init listeners
            self.pos_sub = rospy.Subscriber("/cf6/log_pos", GenericLogData,
                                            self.pos_parser)

        self.pos_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                        self.grid_parser)

        self.fig.show()
        self.fig.canvas.draw()

    def pos_parser(self, msg):
        pos_header = msg.header
        pos_val = msg.values

        drone_id = pos_header.frame_id.split("/")[0] # Extract drone name from topic name
        # Store drone position and convert it from [m] to [cm]
        self.drones_pos_list[drone_id] = drone_pos(pos_header.stamp.secs,
                                                   self.takeofpos[drone_id][0]+(pos_val[0]*m_to_cm),
                                                   self.takeofpos[drone_id][1]+(pos_val[1]*m_to_cm),
                                                   self.takeofpos[drone_id][2]+(pos_val[2]*m_to_cm), None, 0)
        # There is no need to change matrix' values acoording to drones position,
        # because it is already done in Grid module.
        # Drones plotting is done via the grid parser (in order to avoid calling the plot function too many times).

    def grid_parser(self, msg):
        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)
        self.last_matrix = self.matrix
        self.matrix = np.array(msg.data).reshape((grid_height, grid_width))
        keys = list(self.drones_pos_list.keys())
        for iDrone in range(self.nDrones):
            curr_drone_key = keys[iDrone]
            pos = self.drones_pos_list[curr_drone_key]

            virtual_target_pos = []
            self.update_drone_plot([pos.x, pos.y], virtual_target_pos, pos.index)

        self.update_grid_plot()

    # TODO: plot POI

    # Update the grid plot using the last receive matrix (i.e. occupancy grid)
    def update_grid_plot(self):
        diff_matrix = np.subtract(self.matrix, self.last_matrix)
        nonzero = np.nonzero(diff_matrix)
        nonzero_rows = nonzero[0]
        nonzero_cols = nonzero[1]
        for i_nz in range(len(nonzero_rows)):
            i = nonzero_rows[i_nz]
            j = nonzero_cols[i_nz]
            if self.matrix[i][j] == 1:
                self.change_tail_to_empty(i, j)
            elif self.matrix[i][j] == 2:
                self.change_tail_to_wall(i, j)


    def plot_interesting_points(self, interesting_points_list_ij):
        self.erase_interesting_points(self.prev_interesting_points)
        for i, j in interesting_points_list_ij:
            self.change_tail_color_ij( i, j, 'm')
            self.prev_interesting_points.append([i, j])

    def plot_corner_points(self, corner_points_list_ij):
        self.erase_corner_points(self.prev_corner_points)
        for i, j in corner_points_list_ij:
            self.change_tail_color_ij(i, j, 'g')
            self.prev_corner_points.append([i, j])

    def erase_interesting_points(self, interesting_points_list_ij):
        for i, j in interesting_points_list_ij:
            self.change_tail_to_empty(i, j)

    def erase_corner_points(self, corner_points_list_ij):
        for i, j in corner_points_list_ij:
            self.change_tail_to_empty(i, j)

    def plot_border(self):
        border_polygon_patch = PolygonPatch(self.border_polygon, facecolor='white')
        self.ax_env.add_patch(border_polygon_patch)

    def plot_obs_array(self):
        for obs in self.obs_array:
            border_polygon_patch = PolygonPatch(obs, facecolor='orange')
            self.ax_env.add_patch(border_polygon_patch)

    def plot_ij(self, i, j):
        pol_center = self.ij_to_xy(i, j)
        tail = Polygon([(pol_center[0] - self.res / 2, pol_center[1] - self.res / 2),
                        (pol_center[0] - self.res / 2, pol_center[1] + self.res / 2)
                           , (pol_center[0] + self.res / 2, pol_center[1] + self.res / 2),
                        (pol_center[0] + self.res / 2, pol_center[1] - self.res / 2)
                           , (pol_center[0] - self.res / 2, pol_center[1] - self.res / 2)])
        return self.ax_grid.add_patch(PolygonPatch(tail, facecolor='gray'))

    def plot_grid(self):
        for i in range(0, self.matrix.__len__()):
            handles_list = list()
            for j in range(0, self.matrix[i].__len__()):
                handles_list.append(self.plot_ij(i, j))
            self.tail_handles.append(handles_list)

    def change_tail_color_ij(self, i, j, color):
        self.tail_handles[i][j].set_fc(color)

    def change_tail_to_empty(self, i, j):
        self.change_tail_color_ij(i, j, 'k')

    def change_tail_to_wall(self, i, j):
        self.change_tail_color_ij(i, j, 'w')

    def change_tail_list_color(self, tail_list, color):
        for i, j in tail_list:
            self.change_tail_color_ij(i, j, color)

    def plot_points_on_tails_in_list(self, tail_list, color):
        handle_list = list()
        for i, j in tail_list:
            x, y = self.ij_to_xy(i, j)
            h = self.ax_grid.plot(x, y, color)
            handle_list.append(h)
        return handle_list

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def update_drone_plot(self, real_target, virtual_target, list_idx):
        self.plot_handle_envs[list_idx].set_data(real_target[0], real_target[1])
        self.plot_handle_grids[list_idx].set_data(real_target[0], real_target[1])
        # self.plot_handle_envs_vt[drone_idx].set_data(virtual_target[0][0], virtual_target[0][1])
        # self.plot_handle_grids_vt[drone_idx].set_data(virtual_target[0][0], virtual_target[0][1])
        # self.plot_handle_envs_l2vt[drone_idx].set_data([real_target[0][0], virtual_target[0][0]], [real_target[0][1], virtual_target[0][1]])
        # self.plot_handle_grids_l2vt[drone_idx].set_data([real_target[0][0], virtual_target[0][0]], [real_target[0][1], virtual_target[0][1]])
        self.fig.canvas.draw()

    def plot_edges(self, neighbors_list, d_pos, drone_idx):
        self.delete_edges(drone_idx)
        edges = []
        for pos in neighbors_list:
            edge = self.ax_grid.plot([d_pos[0][0], d_pos[0][0]+pos[0][0]], [d_pos[0][1], d_pos[0][1]+pos[0][1]])
            edges.append(edge)
        self.edg_to_neighbors_plot_handles[drone_idx] = edges

    def delete_edges(self, drone_idx):
        for edge_handle in self.edg_to_neighbors_plot_handles[drone_idx]:
            edge_handle[0].remove()
        self.edg_to_neighbors_plot_handles[drone_idx] = []

if __name__ == "__main__":

    rospy.init_node("display_manager")

    env_lim = rospy.get_param("~env_lim")
    env_space = rospy.get_param("~env_space")
    resolution = rospy.get_param("~resolution")

    exec ("env_lim = {}".format(env_lim))

    x_lim = (env_lim[0] - env_space, env_lim[1] + env_space)
    y_lim = (env_lim[2] - env_space, env_lim[3] + env_space)

    polygon_border = [(x_lim[0], y_lim[0]), (x_lim[1], y_lim[0]), (x_lim[1], y_lim[1]),
                      (x_lim[0], y_lim[1]), (x_lim[0], y_lim[0])]

    obs_array = []

    nObstacles = rospy.get_param("~nObstacles")
    for iObstacle in range(nObstacles):
        curr_polygon = rospy.get_param("~obs_{}".format(iObstacle))
        exec("curr_polygon = {}".format(curr_polygon))
        obs_array.append(Polygon(curr_polygon))

    nDrones = rospy.get_param("~nDrones")
    initial_pos_dict = dict()
    for iDrone in range(nDrones):
        curr_drone_name = rospy.get_param("~drone_name_{}".format(iDrone))
        curr_drone_takeoff_pos = rospy.get_param("~drone_takeoff_position_{}".format(iDrone))
        exec ("curr_drone_takeoff_pos = {}".format(curr_drone_takeoff_pos))
        initial_pos_dict[curr_drone_name] = curr_drone_takeoff_pos

    print initial_pos_dict

    matrix = np.zeros([np.int64(np.ceil((x_lim[1] - x_lim[0]) / resolution)),
                            np.int64(np.ceil((y_lim[1] - y_lim[0]) / resolution))])

    display_manager = Display_manager(polygon_border, obs_array, x_lim, y_lim, resolution, matrix, initial_pos_dict, 1)

    plt.show(block=True)