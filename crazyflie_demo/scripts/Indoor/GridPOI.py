#!/usr/bin/env python
import numpy as np
import rospy


class GridPOI:

    def __init__(self, res, env_limits):
        self.res = res
        self.x_lim = env_limits[0:2]
        self.y_lim = env_limits[2:4]
        self.convstrct = np.ones([3, 3])

    def find_POI(self, matrix):
        # temp_mat = copy.deepcopy(matrix)
        # bin_matrix = ndimage.binary_dilation(temp_mat, self.convstrct).astype(temp_mat.dtype)

        # import matplotlib.pyplot as plt
        # fig = plt.figure(45645)
        # ax_1, ax_2 = fig.subplots(1, 2)
        # ax_1.imshow(np.transpose(matrix), origin='lower')
        # ax_2.imshow(np.transpose(bin_matrix), origin='lower')

        interesting_points_list_ij, interesting_points_list_xy = self.find_interesting_points(matrix)
        # rospy.logdebug("POI:{}".format(interesting_points_list_xy))
        corner_points_list_ij, corner_points_list_xy = self.find_corner_points(matrix)

        return [interesting_points_list_ij, interesting_points_list_xy, corner_points_list_ij, corner_points_list_xy]

    def find_interesting_points(self, matrix):
        interesting_points_list_ij = []
        interesting_points_list_xy = []
        tails_list = self.find_interesting_tail(matrix)
        for i, j in tails_list:
            interesting_points_list_ij.append([i, j])
            interesting_points_list_xy.append(self.ij_to_xy(i, j))
        return interesting_points_list_ij, interesting_points_list_xy

    def find_corner_points(self, matrix):
        corner_points_list_ij = []
        corner_points_list_xy = []
        tails_list = self.find_corners_tails(matrix)
        for i, j in tails_list:
            corner_points_list_ij.append([i, j])
            corner_points_list_xy.append(self.ij_to_xy(i, j))
        return corner_points_list_ij, corner_points_list_xy

    def find_corners_tails(self, matrix):
        tail_list = list()
        for i in range(1, matrix.__len__() - 2):
            for j in range(1, matrix[i].__len__() - 2):
                cflag, add_num = self.is_tail_in_corner(i, j, matrix)
                if cflag:
                    cidx = list(np.add([i, j], add_num))
                    if matrix[cidx[0]][cidx[1]] == 1:
                        tail_list.append(cidx)
                    else:
                        tail_list.append([i, j])
        return tail_list

    # def find_corners_tails(self, matrix):
    #     tail_list = list()
    #     for i in range(1, matrix.__len__()-1):
    #         for j in range(1, matrix[i].__len__()-1):
    #             if self.is_tail_in_corner(i, j, matrix):
    #                 tail_list.append([i, j])
    #     return tail_list

    def find_interesting_tail(self, matrix):
        tail_list = list()
        sequence_cnt = np.zeros(np.shape(matrix))
        for i in range(1, matrix.__len__() - 2):
            for j in range(1, matrix[i].__len__() - 2):
                if self.is_tail_interesting(i, j, matrix):
                    # if sequence_cnt[i - 1][j - 1] == 0 and sequence_cnt[i][j - 1] == 0 and sequence_cnt[i - 1][j] == 0:
                    #     tail_list.append([i, j])
                    #     sequence_cnt[i][j] = 1
                    tail_list.append([i, j])
        return tail_list

    def is_tail_interesting(self, i, j, matrix):
        # # Tail is explored
        if i < 1 or i >= matrix.shape[0] or j < 1 or j >= matrix.shape[1]:
            return False
        # Tail on the edge of explored area
        ind_list = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]]
        cnt_wall = 0
        cnt_unexplored = 0
        for k in range(len(ind_list)):
            # if (matrix[i][j] == 1) and (matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 2):
            #     cnt_wall = cnt_wall + 1
            if (matrix[i][j] == 1) and (matrix[ind_list[k][0] + i + ind_list[k][0]][ind_list[k][1] + j + ind_list[k][1]] == 2):
                cnt_wall = cnt_wall + 1
            if (matrix[i][j] == 1) and (matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 0):
                cnt_unexplored = cnt_unexplored + 1
        # if ((cnt_wall == 4) and (cnt_unexplored == 1)):
        #     return False
        if ((cnt_unexplored >= 1) and (cnt_wall >= 1)):
            return True
        else:
            return False

    # def is_tail_in_corner(self, i, j, matrix):
    #     if matrix[i][j] == 1:
    #         if (matrix[i + 1][j] == 1 and matrix[i][j + 1] == 1 and matrix[i + 1][j + 1] != 1 ) or (
    #                 matrix[i + 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i + 1][j - 1] != 1 ) or (
    #                 matrix[i - 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i - 1][j - 1] != 1 ) or (
    #                 matrix[i - 1][j] == 1 and matrix[i][j + 1] == 1 and matrix[i - 1][j + 1] != 1 ):
    #             return True
    #     return False

    def is_tail_in_corner(self, i, j, matrix):
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

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y
