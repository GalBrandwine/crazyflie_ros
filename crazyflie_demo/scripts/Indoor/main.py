# **************** main *******************
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
import numpy as np
import time
from EnvSim import Env
from Grid import Grid
from Agent import Agent
from Drone import Drone
import PathBuilder
from GridPOI import GridPOI
from Display import Display_manager
import copy
import rospy
from std_msgs.msg import String


sleep_time = 0.01
num_of_agents = 5
count_time_step = 0
time_multiplier = 5
num_of_iter = 1000

env = Env(10, 2)
grid = Grid(env.border_polygon_for_grid, 10)
grid_poi = GridPOI(grid.res, grid.x_lim, grid.y_lim)

agents = list()
drones = list()
drones_pos_list = list()
for i in range(0, num_of_agents):
    drone_pos = [[-10000, -10000]]
    while (env.is_in_obs(drone_pos)) or not(env.is_in_border_polygon(drone_pos)):
        drone_pos = env.enterence + 30 * 2 * ([[0.5, 0.5]] - np.random.rand(1, 2))

    drones.append(Drone(i, drone_pos, 0, env))
    agents.append(Agent(i, drone_pos, grid.res, grid.x_lim, grid.y_lim))
    drones_pos_list.append(list(drone_pos[0]))

display = Display(env.border_polygon, env.obs_array, grid.x_lim, grid.y_lim, grid.res, grid.matrix, drones_pos_list)

movie_flag = False
if not movie_flag:
    for t in range(1, num_of_iter):
        count_time_step += 1
        if np.mod(count_time_step, time_multiplier) == 0:

            grid_poi.find_POI(grid.matrix)
            corner_points_list_xy = copy.deepcopy(grid_poi.corner_points_list_xy)
            interesting_points_list_xy = copy.deepcopy(grid_poi.interesting_points_list_xy)

            for i in range(0, num_of_agents):
                Astar_Movement, corner_points_list_xy, interesting_points_list_xy = \
                    PathBuilder.build_trj(drones[i].pos, drones[i].scanning_range, grid.x_lim,
                                          grid.y_lim, grid.res, grid.matrix, corner_points_list_xy, interesting_points_list_xy, drones[drones[i].RefID].pos)
                agents[i].astar_path = Astar_Movement[0]

        for i in range(0, num_of_agents):

            tof_list = drones[i].tof_sensing()
            grid.update_from_tof_sensing_list(tof_list)
            drones[i].preform_step(drones)
            agents[i].preform_step_sys_sim(drones[i].pos, drones[i].current_heading, drones[i].neighbors_pos, grid.matrix)
            drones[i].update_virtual_targets(agents[i].next_pos, agents[i].next_heading)
            display.plot_step(agents[i].next_pos, grid.empty_idxs, grid.wall_idxs, agents[i].reduced_neigbours_pos_list,
                              drones[i].pos, i, grid_poi.interesting_points_list_ij, grid_poi.corner_points_list_ij, grid_poi.wall_idxs_ij)

        # for i in range(0, num_of_agents):
        #     drones[i].stop_command = False

        display.fig.canvas.draw()
        time.sleep(sleep_time)

else: # Creating a movie
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=10, metadata=metadata)
    with writer.saving(display.fig, "writer_test.mp4", 100):

            writer.grab_frame()

