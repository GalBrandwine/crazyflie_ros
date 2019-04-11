import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
from descartes import PolygonPatch
import numpy as np
import matplotlib.animation as manimation
import rospy
from std_msgs.msg import String

class Display:
    def __init__(self, border_polygon, obs_array, x_lim, y_lim, res, matrix, init_pos):

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.matrix = matrix
        self.border_polygon = border_polygon
        self.obs_array = obs_array

        self.fig = plt.figure()
        mgr = plt.get_current_fig_manager()
        mgr.full_screen_toggle()

        ax_env, ax_grid = self.fig.subplots(1, 2)
        plt.axis([self.x_lim[0], self.x_lim[1], self.y_lim[0], self.y_lim[1]])

        self.ax_env = ax_env
        self.ax_grid = ax_grid

        self.tail_handles = list()
        self.corner_plot_handel = list()
        self.interest_plot_handel = list()

        self.plot_grid()
        self.plot_obs_array()

        self.plot_color = 'ob'

        self.plot_handel_envs = []
        self.plot_handel_grids = []
        self.plot_handel_envs_vt = []
        self.plot_handel_grids_vt = []
        self.plot_handel_envs_l2vt = []
        self.plot_handel_grids_l2vt = []
        self.edg_to_neighbors_plot_handels = []
        self.prev_interesting_points = []
        self.prev_corner_points = []

        for pos in init_pos:
            plot_handel_env, = ax_env.plot(pos[0], pos[1], 'ob')
            self.plot_handel_envs.append(plot_handel_env, )
            plot_handel_grid, = ax_grid.plot(pos[0], pos[1], 'ob')
            self.plot_handel_grids.append(plot_handel_grid, )
            plot_handel_env_vt, = ax_env.plot(pos[0], pos[1], 'or')
            self.plot_handel_envs_vt.append(plot_handel_env_vt, )
            plot_handel_grid_vt, = ax_grid.plot(pos[0], pos[1], 'or')
            self.plot_handel_grids_vt.append(plot_handel_grid_vt, )
            plot_handel_env_l2vt, = ax_env.plot(list(np.repeat(pos[0], 2)), list(np.repeat(pos[1], 2)), '--r')
            self.plot_handel_envs_l2vt.append(plot_handel_env_l2vt, )
            plot_handel_grid_l2vt, = ax_grid.plot(list(np.repeat(pos[0], 2)), list(np.repeat(pos[1], 2)), '--r')
            self.plot_handel_grids_l2vt.append(plot_handel_grid_l2vt, )

            self.edg_to_neighbors_plot_handels.append([])

        self.fig.show()
        self.fig.canvas.draw()


    def plot_step(self, virtual_target_pos, empty_idxs, wall_idxs, neighbors_list, drone_pos, drone_idx, interesting_points_list_ij, corner_points_list_ij, wall_corner_idx):
        for tail in empty_idxs:
            self.change_tail_to_empty(tail[0],tail[1])
        for tail in wall_idxs:
            self.change_tail_to_wall(tail[0],tail[1])
        for tail in wall_corner_idx:
            self.change_tail_to_wall(tail[0], tail[1])
        self.plot_interesting_points(interesting_points_list_ij)
        self.plot_corner_points(corner_points_list_ij)
        self.update_drone_plot(drone_pos, virtual_target_pos, drone_idx)
        self.plot_edges(neighbors_list, drone_pos, drone_idx)


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
        handel_list = list()
        for i, j in tail_list:
            x, y = self.ij_to_xy(i, j)
            h = self.ax_grid.plot(x, y, color)
            handel_list.append(h)
        return handel_list

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def update_drone_plot(self, real_target, virtual_target, drone_idx):

        self.plot_handel_envs[drone_idx].set_data(real_target[0][0], real_target[0][1])
        self.plot_handel_grids[drone_idx].set_data(real_target[0][0], real_target[0][1])
        self.plot_handel_envs_vt[drone_idx].set_data(virtual_target[0][0], virtual_target[0][1])
        self.plot_handel_grids_vt[drone_idx].set_data(virtual_target[0][0], virtual_target[0][1])
        self.plot_handel_envs_l2vt[drone_idx].set_data([real_target[0][0], virtual_target[0][0]], [real_target[0][1], virtual_target[0][1]])
        self.plot_handel_grids_l2vt[drone_idx].set_data([real_target[0][0], virtual_target[0][0]], [real_target[0][1], virtual_target[0][1]])

    def plot_edges(self, neighbors_list, drone_pos, drone_idx):
        self.delete_edges(drone_idx)
        edges = []
        for pos in neighbors_list:
            edge = self.ax_grid.plot([drone_pos[0][0], drone_pos[0][0]+pos[0][0]], [drone_pos[0][1], drone_pos[0][1]+pos[0][1]])
            edges.append(edge)
        self.edg_to_neighbors_plot_handels[drone_idx] = edges

    def delete_edges(self, drone_idx):
        for edge_handel in self.edg_to_neighbors_plot_handels[drone_idx]:
            edge_handel[0].remove()
        self.edg_to_neighbors_plot_handels[drone_idx] = []