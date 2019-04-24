import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String


class Drone:
    def __init__(self, AgentID, pos, heading_ang, env):
        self.ID = AgentID
        if AgentID == 0:
            self.RefID = AgentID
        else:
            self.RefID = AgentID - 1
        self.env = env
        self.is_alive = True
        self.acc_factor = 5#10
        self.ang_vel_fac = 5#10
        self.acc_lim = 100 #[cm^2/sec]
        self.vel_lim = 100 #[cd/sec]
        self.ang_vel_lim = np.pi #[rad/sec]
        self.step_noise_size = 20
        self.stepSizeLimit = 30
        self.virtual_target = pos
        self.pos = pos
        self.virtual_heading = heading_ang
        self.current_heading = heading_ang
        self.vel = [0, 0]   #[cm/sec X 2]
        self.ang_vel = 0    #[cd/sec]
        self.scanning_range = 200 #[cm]
        self.neighbours_visibility_range = 400 #cm
        self.dt = 0.2#0.1 #[sec]
        self.stop_command = False
        self.neighbors_pos = list()

    def update_position(self):
        next_pos = np.add(self.pos, np.multiply(self.vel, self.dt))
        if not self.env.is_in_border_polygon(next_pos):
            return
        if not self.env.is_in_obs(next_pos):
            self.pos = next_pos
        # if self.env.is_los(self.pos, next_pos):
        #     self.pos = next_pos
        # else:
        #     flag, int_point = self.env.is_in_obs(self.pos, next_pos)
        #     if flag:
        #         self.pos = int_point
        #     #else: # Note that we have a problem

    def update_velocity(self):
        des_vel = np.subtract(self.virtual_target, self.pos)
        acc = np.multiply(self.acc_factor, (des_vel - self.vel))
        acc_norm = np.linalg.norm(acc)
        if acc_norm > self.acc_lim:
            acc = np.multiply(acc / acc_norm, self.acc_lim)
        vel = self.vel + acc*self.dt
        vel_norm = np.linalg.norm(vel)
        if vel_norm > self.vel_lim:
            self.vel = np.multiply(vel / vel_norm, self.vel_lim)
        else:
            self.vel = vel

    def update_ang(self):
        self.current_heading = self.virtual_heading

    def update_state(self):
        self.update_position()
        self.update_velocity()
        self.update_ang()

    def update_virtual_targets(self, virtual_target, virtual_heading_target):
        self.virtual_target = virtual_target
        self.virtual_heading = virtual_heading_target

    def tof_sensing(self):
        output_list = list()
        directions_vec = np.add([0, np.pi/2, np.pi, 3*np.pi/2], self.current_heading)
        for phi in directions_vec:
            flag, sense_pos = self.env.TOF_sensing(self.pos, phi)
            output_list.append([flag, self.pos, sense_pos])
        return output_list

    def neighbors_sensing(self, drones_arr):
        neighbors_pos = list()
        for i in range(0, drones_arr.__len__()):
            if drones_arr[i].ID != self.ID:
                diff = np.subtract(drones_arr[i].pos, self.pos)
                if np.linalg.norm(diff) < self.neighbours_visibility_range and self.env.is_los(drones_arr[i].pos, self.pos):
                    neighbors_pos.append(diff)
        return neighbors_pos

    def preform_step(self, drone_arr):
        # self.prevent_collision(drone_arr)
        # if not self.stop_command:
        self.update_position()
        self.update_velocity()
        self.update_ang()
        self.neighbors_pos = self.neighbors_sensing(drone_arr)

    def prevent_collision(self, drones):
        for i in range(len(drones)):
            s_next_pos = np.add(self.pos, np.multiply(self.vel, self.dt))
            d_next_pos = np.add(drones[i].pos, np.multiply(drones[i].vel, drones[i].dt))
            if (self.ID != drones[i].ID) and (
                    np.linalg.norm(d_next_pos - s_next_pos) < (self.step_noise_size)) and (
                    drones[i].stop_command != True):
                self.stop_command = True