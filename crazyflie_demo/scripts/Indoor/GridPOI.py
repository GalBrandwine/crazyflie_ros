#!/usr/bin/env python
import numpy as np


class GridPOI:
    """ This module is a part of Cj_injector. It is responsible for finding interesting point (goals of each drone)

        and the corners (point of movement for each drone).

        Possible variable in matrix:
            0 - unexplored area
            1 - explored empty area
            2 - explored area (wall)
    """

    def __init__(self, res, env_limits):
        self.res = res
        self.x_lim = env_limits[0:2]
        self.y_lim = env_limits[2:4]
        self.convstrct = np.ones([3, 3])
        self.wall_dist = 1
        self.Tails_from_edge_to_ignored = 3

    def find_POI(self, matrix):
        """Function for finding Point of Interest.

        :param matrix
        :returns (curners_list_in IJ, curners_list_in XY).

        """
        interesting_points_list_ij, interesting_points_list_xy = self.__find_interesting_points(matrix)
        corner_points_list_ij, corner_points_list_xy = self.__find_corner_points(matrix)

        return [interesting_points_list_ij, interesting_points_list_xy, corner_points_list_ij, corner_points_list_xy]

    # Private methods
    def __find_interesting_points(self, matrix):
        interesting_points_list_ij = []
        interesting_points_list_xy = []
        tails_list = self.__find_interesting_tail(matrix)
        for i, j in tails_list:
            interesting_points_list_ij.append([i, j])
            interesting_points_list_xy.append(self.__ij_to_xy(i, j))
        return interesting_points_list_ij, interesting_points_list_xy

    def __find_corner_points(self, matrix):
        corner_points_list_ij = []
        corner_points_list_xy = []
        tails_list = self.__find_corners_tails(matrix)
        for i, j in tails_list:
            corner_points_list_ij.append([i, j])
            corner_points_list_xy.append(self.__ij_to_xy(i, j))
        return corner_points_list_ij, corner_points_list_xy

    def __find_corners_tails(self, matrix):
        tail_list = list()
        for i in range(self.Tails_from_edge_to_ignored, matrix.__len__() - self.Tails_from_edge_to_ignored):
            for j in range(self.Tails_from_edge_to_ignored, matrix[i].__len__() - self.Tails_from_edge_to_ignored):
                cflag, add_num = self.__is_tail_in_corner(i, j, matrix)
                if cflag:
                    cidx2 = list(np.add([i, j], np.multiply(add_num, [2, 2])))
                    cidx1 = list(np.add([i, j], add_num))
                    if matrix[cidx2[0]][cidx2[1]] == 1:
                        tail_list.append(cidx2)
                    elif matrix[cidx1[0]][cidx1[1]] == 1:
                        tail_list.append(cidx1)
                    else:
                        tail_list.append([i, j])
        return tail_list

    def __find_interesting_tail(self, matrix):
        tail_list = list()
        sequence_cnt = np.zeros(np.shape(matrix))
        for i in range(self.Tails_from_edge_to_ignored, matrix.__len__() - self.Tails_from_edge_to_ignored):
            for j in range(self.Tails_from_edge_to_ignored, matrix[i].__len__() - self.Tails_from_edge_to_ignored):
                if self.__is_tail_interesting(i, j, matrix):
                    if sequence_cnt[i - 1][j - 1] == 0 and sequence_cnt[i][j - 1] == 0 and sequence_cnt[i - 1][j] == 0:
                        tail_list.append([i, j])
                        sequence_cnt[i][j] = 1
                    # tail_list.append([i, j])
        return tail_list

    def __is_tail_interesting(self, i, j, matrix):
        # Tail is explored
        if i < 1 or i >= matrix.shape[0] or j < 1 or j >= matrix.shape[1]:
            return False
        # Tail on the edge of explored area
        ind_list = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]]
        if matrix[i][j] == 1:
            for k in range(len(ind_list)):
                if matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 0:
                    for ti in range(ind_list[k][0] + i - self.wall_dist, ind_list[k][0] + i + self.wall_dist + 1):
                        for tj in range(ind_list[k][1] + j - self.wall_dist, ind_list[k][1] + j + self.wall_dist + 1):
                            if matrix[ti][tj] == 2:
                                return False
                    return True
        else:
            return False

    def __is_tail_in_corner(self, i, j, matrix):
        if matrix[i][j] == 1:
            if matrix[i + 1][j] == 1 and matrix[i][j + 1] == 1 and matrix[i + 1][j + 1] != 1:
                # return True, [-2, -2]
                return True, [-1, -1]
            elif matrix[i + 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i + 1][j - 1] != 1:
                # return True, [-2, 2]
                return True, [-1, 1]
            elif matrix[i - 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i - 1][j - 1] != 1:
                # return True, [2, 2]
                return True, [1, 1]
            elif matrix[i - 1][j] == 1 and matrix[i][j + 1] == 1 and matrix[i - 1][j + 1] != 1:
                # return True, [2, -2]
                return True, [1, -1]
            else:
                return False, []
        else:
            return False, []

    def __xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def __ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y
