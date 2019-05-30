#!/usr/bin/env python
import numpy as np
import rospy


class GridPOI:
    # This module is a part of Cj_injector. It is responsible for finding interesting point (goals of aech drone)
    # and the corners (point of movement for each drone)
    # 0 - unexplored area
    # 1 - explored empty area
    # 2 - explored area (wall)

    def __init__(self, res, env_limits):
        self.res = res
        self.x_lim = env_limits[0:2]
        self.y_lim = env_limits[2:4]
        self.convstrct = np.ones([3, 3])
        self.wall_dist = 1
        self.Tails_from_edge_to_ignored = 3

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
        for i in range(self.Tails_from_edge_to_ignored, matrix.__len__() - self.Tails_from_edge_to_ignored):
            for j in range(self.Tails_from_edge_to_ignored, matrix[i].__len__() - self.Tails_from_edge_to_ignored):
                cflag, add_num = self.is_tail_in_corner(i, j, matrix)
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
        for i in range(self.Tails_from_edge_to_ignored, matrix.__len__() - self.Tails_from_edge_to_ignored):
            for j in range(self.Tails_from_edge_to_ignored, matrix[i].__len__() - self.Tails_from_edge_to_ignored):
                if self.is_tail_interesting(i, j, matrix):
                    if sequence_cnt[i - 1][j - 1] == 0 and sequence_cnt[i][j - 1] == 0 and sequence_cnt[i - 1][j] == 0:
                        tail_list.append([i, j])
                        sequence_cnt[i][j] = 1
                    # tail_list.append([i, j])
        return tail_list


    def is_tail_interesting(self, i, j, matrix):
        # # Tail is explored
        if i < 1 or i >= matrix.shape[0] or j < 1 or j >= matrix.shape[1]:
            return False
        # Tail on the edge of explored area
        ind_list = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]]
        if (matrix[i][j] == 1):
            for k in range(len(ind_list)):
                if (matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 0):
                    for ti in range(ind_list[k][0] + i - self.wall_dist, ind_list[k][0] + i + self.wall_dist + 1):
                        for tj in range(ind_list[k][1] + j - self.wall_dist, ind_list[k][1] + j + self.wall_dist + 1):
                            if matrix[ti][tj] == 2:
                                return False
                    return True
        else:
            return False


    # def is_tail_interesting(self, i, j, matrix):
    #     # # Tail is explored
    #     if i < 1 or i >= matrix.shape[0] or j < 1 or j >= matrix.shape[1]:
    #         return False
    #     # Tail on the edge of explored area
    #     unexp_val = 1
    #     wall_val = 3
    #     unexp_idxs = [[-unexp_val, -unexp_val], [0, -unexp_val], [unexp_val, -unexp_val], [unexp_val, 0], [unexp_val, unexp_val], [0, unexp_val], [-unexp_val, unexp_val], [-unexp_val, 0]]
    #     wall_idxs = [[-wall_val, -wall_val], [0, -wall_val], [wall_val, -wall_val], [wall_val, 0], [wall_val, wall_val], [0, wall_val], [-wall_val, wall_val], [-wall_val, 0]]
    #     for k in range(len(unexp_idxs)):
    #         if (matrix[i][j] == 1) and (matrix[unexp_idxs[k][0] + i][unexp_idxs[k][1] + j] == 0):
    #             for t in range(len(wall_idxs)):
    #                 try:
    #                     if (matrix[unexp_idxs[k][0] + i + wall_idxs[t][0]][unexp_idxs[k][1] + j + wall_idxs[t][1]] == 2) and \
    #                             (matrix[unexp_idxs[k][0] + i + unexp_idxs[t][0]][unexp_idxs[k][1] + j + unexp_idxs[t][1]] != 2):
    #
    #                         return True
    #                 except:
    #                     continue
    #     return False

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