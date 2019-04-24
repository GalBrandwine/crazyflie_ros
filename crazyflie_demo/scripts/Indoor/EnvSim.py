import numpy as np
from shapely.geometry import Point, LineString, Polygon

class Env:

    def __init__(self, res, maze_case):
        self.obs_array = list()
        self.obs_vertex_list = []
        self.target_handel = []
        self.los_to_target_handel = []
        self.res = res
        self.sensor_range = 300 #cm
        if maze_case == 0:
            self.build_simple_env()
        elif maze_case == 1:
            self.build_corridor()
        elif maze_case == 2:
            self.build_barel_maze()
        elif maze_case == 3:
            self.build_simple_maze()

        x_lim = [self.border_polygon_for_grid[0][0], self.border_polygon_for_grid[0][0]]
        y_lim = [self.border_polygon_for_grid[0][1], self.border_polygon_for_grid[0][1]]
        for i in range(1, self.border_polygon_for_grid.__len__()):
            if x_lim[0] > self.border_polygon_for_grid[i][0]:
                x_lim[0] = self.border_polygon_for_grid[i][0]
            if x_lim[1] < self.border_polygon_for_grid[i][0]:
                x_lim[1] = self.border_polygon_for_grid[i][0]
            if y_lim[0] > self.border_polygon_for_grid[i][1]:
                y_lim[0] = self.border_polygon_for_grid[i][1]
            if y_lim[1] < self.border_polygon_for_grid[i][1]:
                y_lim[1] = self.border_polygon_for_grid[i][1]

    def is_in_border_polygon(self, pos):
        return self.border_polygon.contains(Point(pos[0][0], pos[0][1]))

    def is_in_obs(self, pos):
        for obs in self.obs_array:
            if obs.contains(Point(pos[0][0], pos[0][1])):
                return True
        return False

    def is_step_legal(self, curr_pos, step):
        new_pos = curr_pos + step
        if not self.is_in_border_polygon(new_pos):
            return  False
        return self.is_los(curr_pos, new_pos)

    def is_los(self, p1, p2):
        line = LineString([(p1[0][0], p1[0][1]), (p2[0][0], p2[0][1])])
        for obs in self.obs_array:
            if obs.intersection(line,):
                return False
        return True

    def TOF_sensing(self, sensor_pos, sensor_dir):
        tof_target = sensor_pos + np.multiply(self.sensor_range, [[np.cos(sensor_dir), np.sin(sensor_dir)]])
        intersection_list = list()
        line = LineString([(sensor_pos[0][0], sensor_pos[0][1]), (tof_target[0][0], tof_target[0][1])])
        for obs in self.obs_array:
            inter_obj = obs.intersection(line)
            if not inter_obj.is_empty:
                int_list = list(obs.intersection(line).coords)
                for inter in int_list:
                    intersection_list.append(inter)

        if intersection_list.__len__() == 0:
            return False, tof_target
        else:
            closest_int_norm = self.sensor_range + 1
            closest_int = [[0, 0]]
            for inter in intersection_list:
                int_norm = np.linalg.norm(sensor_pos - inter)
                if int_norm <= closest_int_norm:
                    closest_int_norm = int_norm
                    closest_int = inter
            return True, [[closest_int[0], closest_int[1]]]

    def los_intersection_point(self, p1, p2):
        intersection_list = list()
        line = LineString([(p1[0][0], p1[0][1]), (p2[0][0], p2[0][1])])
        for obs in self.obs_array:
            inter_obj = obs.intersection(line)
            if not inter_obj.is_empty:
                int_list = list(obs.intersection(line).coords)
                for inter in int_list:
                    intersection_list.append(inter)

        if intersection_list.__len__() == 0:
            return False, p2
        else:
            closest_int_norm = self.sensor_range + 1
            closest_int = [[0, 0]]
            for inter in intersection_list:
                int_norm = np.linalg.norm(np.subtract(p1,[[inter[0], inter[1]]]))
                if int_norm <= closest_int_norm:
                    closest_int_norm = int_norm
                    closest_int = inter
            return True, [[closest_int[0], closest_int[1]]]

    def is_los_to_trg(self, pos):
        return self.is_los(self.target_pos, pos)

    def is_detect_trg(self, pos):
        if self.target_alive:
            if self.is_los_to_trg(pos):
                return True
        return False

    def build_wall(self, nw, se):
        # The polygon coordinate is from Barel's xls, in cell indices
        return [(nw[0]*self.res, nw[1]*self.res), (nw[0]*self.res, se[1]*self.res), (se[0]*self.res, se[1]*self.res), (se[0]*self.res, nw[1]*self.res), (nw[0]*self.res, nw[1]*self.res)]

    def build_barel_maze(self):

        self.enterence = [[345, 54]]
        self.target_pos = [[420, 340]]
        self.target_alive = True
        self.is_target_mang = False
        self.border_polygon_for_grid = [(0*self.res, 0*self.res), (57*self.res, 0*self.res), (57*self.res, 55*self.res), (0*self.res, 55*self.res), (0*self.res, 0*self.res)]
        self.border_polygon = Polygon(self.border_polygon_for_grid)
        obs_list = list()
        # The wall coordinate is from Barel's xls, in cell indices
        obs_list.append(self.build_wall([0, 0], [1, 55]))
        obs_list.append(self.build_wall([1, 0], [31, 1]))
        obs_list.append(self.build_wall([1, 48], [16, 49]))
        obs_list.append(self.build_wall([1, 54], [56, 55]))
        obs_list.append(self.build_wall([6, 12], [11, 13]))
        obs_list.append(self.build_wall([10, 1], [11, 6]))
        obs_list.append(self.build_wall([10, 13], [11, 23]))
        obs_list.append(self.build_wall([12, 28], [13, 43]))
        obs_list.append(self.build_wall([12, 43], [17, 44]))
        obs_list.append(self.build_wall([16, 44], [17, 54]))
        obs_list.append(self.build_wall([17, 1], [18, 21]))
        obs_list.append(self.build_wall([28, 11], [29, 20]))
        obs_list.append(self.build_wall([20, 27], [21, 37]))
        obs_list.append(self.build_wall([21, 27], [31, 28]))
        obs_list.append(self.build_wall([21, 36], [31, 37]))
        obs_list.append(self.build_wall([18, 20], [29, 21]))
        obs_list.append(self.build_wall([28, 1], [29, 6]))
        obs_list.append(self.build_wall([21, 44], [22, 54]))
        obs_list.append(self.build_wall([22, 44], [27, 45]))
        obs_list.append(self.build_wall([31, 27], [32, 37]))
        obs_list.append(self.build_wall([32, 44], [33, 54]))
        obs_list.append(self.build_wall([36, 0], [57, 1]))
        obs_list.append(self.build_wall([39, 1], [40, 21]))
        obs_list.append(self.build_wall([39, 28], [40, 39]))
        obs_list.append(self.build_wall([39, 44], [40, 54]))
        obs_list.append(self.build_wall([40, 28], [49, 29]))
        obs_list.append(self.build_wall([45, 29], [46, 44]))
        obs_list.append(self.build_wall([45, 49], [46, 54]))
        obs_list.append(self.build_wall([46, 9], [56, 10]))
        obs_list.append(self.build_wall([46, 20], [56, 21]))
        obs_list.append(self.build_wall([46, 40], [56, 41]))
        obs_list.append(self.build_wall([56, 1], [57, 55]))
        for obs in obs_list:
            self.obs_array.append(Polygon(obs))
            for vertex in obs:
                self.obs_vertex_list.append(vertex)
            del self.obs_vertex_list[-1]

    def build_simple_env(self):
        self.enterence = [[0, 0]]
        self.target_pos = [[1, 1]]
        self.target_alive = False
        self.is_target_mang = False
        self.border_polygon_for_grid = [(-100, -100), (100, -100), (100, 100), (-100, 100), (-100, -100)]
        self.border_polygon = Polygon(self.border_polygon_for_grid)
        obs_array = list()
        for obs in obs_array:
            self.obs_array.append(Polygon(obs))
            for vertex in obs:
                self.obs_vertex_list.append(vertex)
            del self.obs_vertex_list[-1]

    def build_simple_maze(self):
        self.enterence = [[-30, 0]]
        self.target_pos = [[1, 1]]
        self.target_alive = False
        self.is_target_mang = False
        self.border_polygon_for_grid = [(-45, -40), (40, -40), (40, 40), (-40, 40), (-40, -40)]
        self.border_polygon = Polygon(self.border_polygon_for_grid)
        obs_array = list()
        obs_array.append([(-40, -10), (-35, -10), (-35, -40), (-40, -40), (-40, -10)])
        obs_array.append([(-40, 10), (-35, 10), (-35, 40), (-40, 40), (-40, 10)])
        obs_array.append([(-10, -10), (20, -10), (20, -5), (-10, -5), (-10, -10)])
        obs_array.append([(20, -5), (20, 20), (15, 20), (15, -5), (20, -5)])
        obs_array.append([(-10, 5), (-10, 20), (-5, 20), (-5, 5), (-10, 5)])
        obs_array.append([(-10, 20), (20, 20), (20, 25), (-10, 25), (-10, 20)])
        obs_array.append([(-40, -40), (-40, -35), (40, -35), (40, -40), (-40, -40)])
        obs_array.append([(-40, 40), (-40, 35), (40, 35), (40, 40), (-40, 40)])
        obs_array.append([(40, 40), (35, 40), (35, -40), (40, -40), (40, 40)])
        for obs in obs_array:
            self.obs_array.append(Polygon(obs))
            for vertex in obs:
                self.obs_vertex_list.append(vertex)
            del self.obs_vertex_list[-1]

    def build_corridor(self):
        self.enterence = [[-30, 0]]
        self.target_pos = [[54, 0]]
        self.target_alive = True
        self.is_target_mang = False
        self.border_polygon_for_grid = [(-60, -60), (60, -60), (60, 60), (-60, 60), (-60, -60)]
        self.border_polygon = Polygon(self.border_polygon_for_grid)
        obs_array = list()
        obs_array.append([(-2.5, -5), (2.5, -5), (2.5, -60), (-2.5, -60), (-2.5, -5)])
        obs_array.append([(-2.5, 5), (2.5, 5), (2.5, 60), (-2.5, 60), (-2.5, 5)])
        obs_array.append([(-60, -60), (-60, -55), (60, -55), (60, -60), (-60, -60)])
        obs_array.append([(-60, 60), (-60, 55), (60, 55), (60, 60), (-60, 60)])
        obs_array.append([(60, 60), (55, 60), (55, -60), (60, -60), (60, 60)])
        obs_array.append([(-60, 60), (-55, 60), (-55, -60), (-60, -60), (-60, 60)])

        for obs in obs_array:
            self.obs_array.append(Polygon(obs))
            for vertex in obs:
                self.obs_vertex_list.append(vertex)
            del self.obs_vertex_list[-1]

