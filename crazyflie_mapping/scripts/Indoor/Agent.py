#!/usr/bin/env python
import math

import numpy as np


class Agent:
    """ Class for controlling the drone.

        Each drone has This module chooses the next position for each drone using the function Dynam_Search_in_maze

    0 - unexplored area
    1 - explored empty area
    2 - explored area (wall)

    """
    def __init__(self, AgentID, pos, res, env_limits):
        self.ID = AgentID
        self.agent_alive = True
        self.is_homing = False
        self.velocityFactor = 50
        self.step_noise_size = 20
        self.step_snr = 1
        self.stepSizeLimit = 50
        self.step_factor = 1
        self.next_pos = pos
        self.current_pos = self.next_pos
        self.next_heading = math.radians(0)
        self.current_heading = self.next_heading
        self.VisibilityRange = 300
        self.scanning_range = 200
        self.repulse_range = self.VisibilityRange / 10
        self.pull_range = self.VisibilityRange * 4 / 5
        self.goal_orianted_flag_flip_prob = 0
        self.goal_orianted_flag = True  # np.random.rand(1) < self.goal_orianted_flag_flip_prob
        self.reduced_neigbours_pos_list = list()
        self.astar_path = []
        self.x_lim = env_limits[0:2]
        self.y_lim = env_limits[2:4]
        self.res = res
        self.dist_factor = 1

    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading

    def get_virtual_target_and_heading(self):
        return self.next_pos, self.next_heading

    def preform_step_sys_sim(self, current_pos, current_heading, matrix, dict_of_drones_pos, tf_prefix):
        self.update_current_state(current_pos, current_heading)
        self.Dynam_Search_in_maze(matrix, dict_of_drones_pos, tf_prefix)

    def Dynam_Search_in_maze(self, matrix, dict_of_drones_pos, tf_prefix):

        max_count_val = 10
        break_counter = 0
        vec = np.zeros(2)
        flag = False
        tails_from_wall = 1
        as_flag = False
        noise_fac = 1
        close_wall = False

        # if self.astar_path == []:
        #     vec = np.subtract(self.next_pos[0], self.current_pos[0])
        # elif(self.is_step_legal(self.current_pos,  np.subtract(self.astar_path[0], self.current_pos[0]), matrix)):
        #     vec = np.subtract(self.astar_path[0], self.current_pos[0])
        #     as_flag = True
        # if self.astar_path != [] and np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size\
        #         and self.is_step_legal(self.current_pos,  np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
        #     vec = np.subtract(self.next_pos[0], self.current_pos[0])

        # If there are steps left in the path and the next step is in line of sight then choose it.
        if self.astar_path != [] and self.is_step_legal(self.current_pos,
                                                        np.subtract(self.astar_path[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.astar_path[0], self.current_pos[0])
            as_flag = True
        # If there are steps left in the path and the previous step is not finished and still legal then resume the prevoius step
        elif self.astar_path != [] and np.linalg.norm(
                np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size \
                and self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])
        # If there are no steps in path and the previous step is still legal then resume the previous step
        elif self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])

        # Check if the choosen step will be to close to a wall
        if sum(vec) != 0:
            ivec, jvec = self.xy_to_ij(vec[0], vec[1])
            for ti in range(ivec - tails_from_wall, ivec + tails_from_wall + 1):
                for tj in range(jvec - tails_from_wall, jvec - tails_from_wall + 1):
                    if matrix[ti][tj] == 2:
                        close_wall = True
                        break
        else:
            close_wall = True

        # If indeed it is to close to a wall then move in the same direction but stop a few tail before the wall
        if close_wall:
            if np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.res:
                step = np.multiply(np.divide(vec, np.linalg.norm(vec)),
                                   np.linalg.norm(vec) - (tails_from_wall * self.res))
                if (np.linalg.norm(vec) - (tails_from_wall * self.res)) > 0:
                    vec = step

            # while break_counter < max_count_val and as_flag == False and sum(vec) != 0:
            #     break_counter = break_counter + 1
            #     step = self.step_noise_size * noise_fac * ([0.5, 0.5] - np.random.rand(2)) + self.current_pos[0]
            #     istep, jstep = self.xy_to_ij(step[0], step[1])
            #     if self.is_step_legal(self.current_pos, step, matrix) and matrix[istep + tails_from_wall][jstep] != 2 and\
            #             matrix[istep - tails_from_wall][jstep] != 2 and matrix[istep][jstep + tails_from_wall] != 2 and\
            #             matrix[istep][jstep - tails_from_wall] != 2:
            #         vec = step
            #         break

        # Limit the step size to maximum distance
        if np.linalg.norm(vec) > self.stepSizeLimit:
            temp = np.divide(vec, np.linalg.norm(vec))
            vec = np.multiply(temp, self.stepSizeLimit)

        if break_counter < max_count_val:
            self.next_pos = self.current_pos + vec
            if as_flag and self.astar_path != []:
                del self.astar_path[0]

    def update_sate(self, pos, heading):
        self.current_pos = pos
        self.current_heading = heading

    def is_step_legal(self, curr_pos, step, matrix):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1] and (
                matrix[i][j] == 1 or matrix[i][j] == 3)):
            return False
        return self.is_los(curr_pos, new_pos, matrix)

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y

    def is_los(self, p1, p2, matrix):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1 - p2) / self.res) * 3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if matrix[i][j] != 1:
                return False
        return True
