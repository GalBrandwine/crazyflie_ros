#!/usr/bin/env python
import numpy as np
from scipy import ndimage
import copy

class GridPOI:

    def __init__(self, res, x_lim, y_lim):
        self.res = res
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.interesting_points_list_ij = []
        self.interesting_points_list_xy = []
        self.corner_points_list_ij = []
        self.corner_points_list_xy = []
        self.wall_idxs_ij = []
        self.wall_idxs_xy = []
        self.convstrct = np.ones([3, 3])

    def find_POI(self, matrix):
        # temp_mat = copy.deepcopy(matrix)
        # bin_matrix = ndimage.binary_dilation(temp_mat, self.convstrct).astype(temp_mat.dtype)

        # import matplotlib.pyplot as plt
        # fig = plt.figure(45645)
        # ax_1, ax_2 = fig.subplots(1, 2)
        # ax_1.imshow(np.transpose(matrix), origin='lower')
        # ax_2.imshow(np.transpose(bin_matrix), origin='lower')

        self.interesting_points_list_ij, self.interesting_points_list_xy = self.find_interesting_points(matrix)
        self.corner_points_list_ij, self.corner_points_list_xy = self.find_corner_points(matrix)
        self.wall_idxs_ij = self.complete_wall_in_corners(matrix)

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
        for i in range(1, matrix.__len__()-1):
            for j in range(1, matrix[i].__len__()-1):
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
        for i in range(1, matrix.__len__()-1):
            for j in range(1, matrix[i].__len__()-1):
                if self.is_tail_interesting(i, j, matrix):
                    if sequence_cnt[i-1][j-1] == 0 and sequence_cnt[i][j-1] == 0 and sequence_cnt[i-1][j] == 0:
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
        cnt_wall = 0
        cnt_unexplored = 0
        for k in range(len(ind_list)):
            if (matrix[i][j] == 1) and (matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 2):
                cnt_wall = cnt_wall + 1
            if (matrix[i][j] == 1) and (matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 0):
                cnt_unexplored = cnt_unexplored + 1
        if ((cnt_wall == 4) and (cnt_unexplored == 1)):
            return False
        if ((cnt_wall >= 1) and (cnt_unexplored >= 1)):# or cnt_unexplored >= 4:
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
                return True, [-1, -1]
            elif matrix[i + 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i + 1][j - 1] != 1:
                return True, [-1, 1]
            elif matrix[i - 1][j] == 1 and matrix[i][j - 1] == 1 and matrix[i - 1][j - 1] != 1:
                return True, [1, 1]
            elif matrix[i - 1][j] == 1 and matrix[i][j + 1] == 1 and matrix[i - 1][j + 1] != 1:
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


    def complete_wall_in_corners(self, matrix):
        wall_idxs_ij = []
        # wall_idxs_xy = []
        for i in range(1, matrix.__len__()-1):
            for j in range(1, matrix[i].__len__()-1):
                if matrix[i][j] == 0:
                    if ((matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1 or matrix[i - 1][j - 1] == 3)) or
                        (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1 or matrix[i - 1][j - 1] == 3)) or
                        (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1 or matrix[i - 1][j + 1] == 3)) or
                        (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1 or matrix[i - 1][j + 1] == 3))):
                        # change_tail_to_wall(i, j) # Originally
                        wall_idxs_ij.append([i,j])
                        # wall_idxs_xy.append([self.ij_to_xy(i, j)])
        j = 0
        for i in range(1, matrix.__len__()-1):
            if (matrix[i][j] == 0 and
                    (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1 or matrix[i + 1][j + 1] == 3)) or
                    (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1 or matrix[i - 1][j + 1] == 3))):
                wall_idxs_ij.append([i, j])
                # wall_idxs_xy.append([self.ij_to_xy(i, j)])

        j = matrix[0].__len__()-1
        for i in range(1, matrix.__len__() - 1):
            if (matrix[i][j] == 0 and
                    (matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1 or matrix[i - 1][j - 1] == 3)) or
                    (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1 or matrix[i + 1][j - 1] == 3))):
                wall_idxs_ij.append([i, j])
                # wall_idxs_xy.append([self.ij_to_xy(i, j)])

        i = 0
        for j in range(1, matrix[0].__len__()-1):
            if (matrix[i][j] == 0 and
                    (matrix[i + 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i + 1][j - 1] == 1 or matrix[i - 1][j - 1] == 3)) or
                    (matrix[i + 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i + 1][j + 1] == 1 or matrix[i + 1][j - 1] == 3))):
                wall_idxs_ij.append([i, j])
                # wall_idxs_xy.append([self.ij_to_xy(i, j)])

        i = matrix.__len__()-1
        for j in range(1, matrix[0].__len__() - 1):
            if (matrix[i][j] == 0 and
                    (matrix[i - 1][j] == 2 and matrix[i][j - 1] == 2 and (matrix[i - 1][j - 1] == 1 or matrix[i - 1][j - 1] == 3)) or
                    (matrix[i - 1][j] == 2 and matrix[i][j + 1] == 2 and (matrix[i - 1][j + 1] == 1 or matrix[i - 1][j + 1] == 3))):
                wall_idxs_ij.append([i, j])
                # wall_idxs_xy.append([self.ij_to_xy(i, j)])

        if (matrix[0][0] == 0 and
                    (matrix[0][1] == 2 and matrix[1][0] == 2)):
                # change_tail_to_wall(0, 0)
                wall_idxs_ij.append([0, 0])
                # wall_idxs_xy.append([self.ij_to_xy(0, 0)])

        if (matrix[0][matrix[0].__len__()-1] == 0 and
                (matrix[0][matrix[0].__len__()-2] == 2 and matrix[1][matrix[0].__len__()-1] == 2)):
            # change_tail_to_wall(0, matrix[0].__len__()-1)
            wall_idxs_ij.append([0, matrix[0].__len__()-1])
            # wall_idxs_xy.append([self.ij_to_xy(0, matrix[0].__len__()-1)])

        if (matrix[matrix.__len__()-1][matrix[0].__len__()-1] == 0 and
                (matrix[matrix[0].__len__()-1][matrix[0].__len__()-2] == 2 and matrix[matrix[0].__len__()-2][matrix[0].__len__()-1] == 2)):
            # change_tail_to_wall(matrix.__len__()-1, matrix[0].__len__()-1)
            wall_idxs_ij.append([matrix.__len__()-1, matrix[0].__len__()-1])
            # wall_idxs_xy.append([self.ij_to_xy(matrix.__len__()-1, matrix[0].__len__()-1)])

        if (matrix[matrix.__len__()-1][0] == 0 and
                (matrix[matrix[0].__len__()-1][1] == 2 and matrix[matrix[0].__len__()-2][0] == 2)):
            # change_tail_to_wall(matrix.__len__()-1, 0)
            wall_idxs_ij.append([matrix.__len__()-1, 0])
            # wall_idxs_xy.append([self.ij_to_xy(matrix.__len__()-1, 0)])

        return wall_idxs_ij


