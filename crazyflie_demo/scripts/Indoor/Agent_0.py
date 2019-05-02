#!/usr/bin/env python
import numpy as np
from math import radians
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf.transformations import euler_from_quaternion
from Grid import drone_pos, m_to_cm
import threading

m_to_cm = 100

class Agent_injector:

    def __init__(self, AgentID, res, x_lim, y_lim, drone_takeoff_pos, drone_name, matrix):

        self.ID = AgentID
        self.matrix = matrix
        self.last_matrix = matrix
        self.drone_name = drone_name
        self.agent_alive = True
        self.is_homing = False
        self.velocityFactor = 50
        self.step_noise_size = 20
        self.step_snr = 1
        self.stepSizeLimit = 30
        self.step_factor = 1
        self.next_pos = self.current_pos
        self.current_pos = [[drone_takeoff_pos[0], drone_takeoff_pos[1]]]
        self.next_heading = 0
        self.current_heading = self.next_heading
        self.VisibilityRange = 300
        self.scanning_range = 200
        self.repulse_range = self.VisibilityRange/10
        self.pull_range = self.VisibilityRange*4/5
        self.goal_orianted_flag_flip_prob = 0
        self.goal_orianted_flag = True #np.random.rand(1) < self.goal_orianted_flag_flip_prob
        self.reduced_neigbours_pos_list = list()
        self.astar_path = []
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.dist_factor = 3
        self.step = 30  # cm
        self.height_thr = 30

        self.takeofpos = curr_drone_takeoff_pos
        self.drones_pos_list = dict()

        self.drones_pos_list[self.drone_name] = drone_pos(time = 0, x = drone_takeoff_pos[0], y = drone_takeoff_pos[1],
                                                          z = drone_takeoff_pos[2], w = None, index = self.ID)

        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser, callback_args="/indoor/occupancy_grid_topic")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Start cj publisher
        cj_pub_thread = threading.Thread(name='cj_pub_thread', target=self.init_cj_publisher)
        cj_pub_thread.start()


    # Initialize a publisher for cj
    def init_cj_publisher(self):
        self.Cj_injector_pub = rospy.Publisher(self.drone_name + "/Cj_injcetor", PoseStamped, queue_size=10)

        rate = rospy.Rate(2)  # 2 Hz
        while not rospy.is_shutdown():

            x = self.next_pos[0]/m_to_cm
            y = self.next_pos[1]/m_to_cm
            z = 35/m_to_cm
            roll = 0
            pitch = 0
            yaw = 0

            # Publish the message
            pose = self.to_pose_stamped(x, y, z, roll, pitch, yaw)
            if self.drones_pos_list[self.drone_name].z > self.height_thr:
                self.Cj_injector_pub.publish(pose)
                rate.sleep()


    def to_pose_stamped(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


    def grid_parser(self, msg, topic):

        drone_id = self.drone_name
        plt_index = self.ID
        try:
            trans = self.tfBuffer.lookup_transform('world', drone_id, rospy.Time(0))

            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)

            euler = euler_from_quaternion(q, axes='sxyz')

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            self.pos = [x, y, z, roll, pitch, yaw]
            # rospy.loginfo("pos in Display: {}\n".format(self.pos))

            # Store drone position and convert it from [m] to [cm]
            self.drones_pos_list[drone_id] = drone_pos(0, x*m_to_cm, y*m_to_cm, z*m_to_cm, yaw, plt_index)

            self.next_pos = [[self.current_pos[0][0], self.current_pos[0][1]]]

        except:
            rospy.logdebug("tf lookup -- {} not found".format(drone_id))
        # except Exception as e:
        #     rospy.loginfo(e)

        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)
        self.last_matrix = self.matrix
        self.matrix = np.array(msg.data).reshape((grid_height, grid_width))


    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading


    def get_virtual_target_and_heading(self):
        return self.next_pos, self.next_heading


    def preform_step_sys_sim(self, current_pos, current_heading, neigbours_pos_list, matrix):
        self.update_current_state(current_pos, current_heading)
        self.reduced_neigbours_pos_list = self.neighborhood_reduction(neigbours_pos_list, matrix)
        self.Dynam_Search_in_maze(self.reduced_neigbours_pos_list, matrix)
        self.next_heading = np.random.rand() * np.pi / 4


    def Dynam_Search_in_maze(self, matrix):

        max_count_val = 10
        break_counter = 0
        vec = np.zeros(2)
        flag = False
        as_flag = False

        if self.astar_path == []:
            vec = [self.next_pos[0][0] - self.current_pos[0][0], self.next_pos[0][1] - self.current_pos[0][1]]
        else:
            vec = [self.astar_path[0][0] - self.current_pos[0][0], self.astar_path[0][1] - self.current_pos[0][1]]
            as_flag = True

        if self.astar_path != [] and np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size\
                and self.is_step_legal(self.current_pos, [self.next_pos[0][0]-self.current_pos[0][0],self.next_pos[0][1]-self.current_pos[0][1]], matrix):
            vec = [self.next_pos[0][0] - self.current_pos[0][0], self.next_pos[0][1] - self.current_pos[0][1]]

        while not flag and break_counter < max_count_val:
            break_counter = break_counter + 1
            step = self.step_noise_size * ([0.5, 0.5] - np.random.rand(2)) + vec
            if self.is_step_legal(self.current_pos, step, matrix):
                flag = True
                break

        if break_counter < max_count_val:
            self.next_pos = self.current_pos + step
            self.attempts_cnt = 0
            if as_flag and self.astar_path != []:
                del self.astar_path[0]


    def non_scaned_list(self, p, r, matrix):
        non_sc_list = list()
        base_unit_vector = [[0, 1]]
        for theta in np.linspace(0, 2*np.pi, num=10, endpoint=False):
            unit_vector = [[base_unit_vector[0][0]*np.cos(theta) - base_unit_vector[0][1]*np.sin(theta),
                            base_unit_vector[0][0] * np.sin(theta) + base_unit_vector[0][1] * np.cos(theta)]]
            for dr in np.linspace(self.res, r, num=r / self.res * 2, endpoint=True):
                p2 = p + [[unit_vector[0][0]*dr, unit_vector[0][1]*dr]]
                i, j = self.xy_to_ij(p2[0][0], p2[0][1])
                if 0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1]:
                    if matrix[i][j] == 2:
                        break
                    else:
                        if matrix[i][j] == 0:
                            non_sc_list.append([self.ij_to_xy(i, j)])
        return non_sc_list


    def is_step_in_corridor(self, step, neighbor_pos, matrix):
        neighbor_abs_pos = self.current_pos + neighbor_pos
        if self.is_step_legal(neighbor_abs_pos, step, matrix):
            neighbor_abs_pos_potential = neighbor_abs_pos + step
        else:
            neighbor_pos_unit = neighbor_pos / np.linalg.norm(neighbor_pos)
            neighbor_step_potential = step - 2 * np.dot(step[0], neighbor_pos_unit[0]) * neighbor_pos_unit
            neighbor_abs_pos_potential = neighbor_abs_pos + neighbor_step_potential

        return self.is_los(self.current_pos + step, neighbor_abs_pos_potential, matrix)


    def outOfLimit_Ando(self, neighbor_pos, step):
        avg_pos = np.divide(neighbor_pos, 2)
        deltas_step = step - avg_pos
        return np.linalg.norm(deltas_step) > self.VisibilityRange/2


    def Sensing(self, agents_arr, matrix):
        neighbors_pos = []
        for i in range(0, agents_arr.__len__()):
            diff = agents_arr[i].current_pos - self.current_pos
            if (agents_arr[i].ID != self.ID and np.linalg.norm(diff) < self.VisibilityRange and
                    self.is_los(agents_arr[i].current_pos, self.current_pos, matrix)):
                neighbors_pos.append(diff)
        return neighbors_pos


    def neighborhood_reduction(self, neighbors_pos, matrix):
        reduced_neighbors_pos = []
        for i in range(0, neighbors_pos.__len__()):
            flag = True
            counter = 0
            for j in range(0, neighbors_pos.__len__()):
                if i != j and ((np.linalg.norm(neighbors_pos[i]) > np.linalg.norm(neighbors_pos[j])) and
                               (np.linalg.norm(neighbors_pos[i]) >
                                np.linalg.norm(neighbors_pos[j] - neighbors_pos[i]))):
                    if self.is_los(self.current_pos + neighbors_pos[i], self.current_pos + neighbors_pos[j], matrix):
                        counter = counter + 1
                        if counter > 0:
                            flag = False
                            break
            if flag:
                reduced_neighbors_pos.append(neighbors_pos[i])
        return reduced_neighbors_pos


    def update_sate(self, pos, heading):
        self.current_pos = pos
        self.current_heading = heading


    def is_step_legal(self, curr_pos, step, matrix):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1] and (matrix[i][j] == 1 or matrix[i][j] == 3)):
            return False
        return self.is_los(curr_pos, new_pos, matrix)


    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def is_los(self, p1, p2, matrix):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1-p2)/self.res)*3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if matrix[i][j] != 1 and matrix[i][j] != 3:
                return False
        return True


if __name__ == "__main__":

    rospy.init_node("agent_incjetor")

    env_lim = rospy.get_param("~env_lim")
    env_space = rospy.get_param("~env_space")
    resolution = rospy.get_param("~resolution")
    drone_name = rospy.get_param("~drone_name_0")
    curr_drone_takeoff_pos = rospy.get_param("~drone_takeoff_position_0")

    exec ("env_lim = {}".format(env_lim))

    x_lim = (env_lim[0] - env_space, env_lim[1] + env_space)
    y_lim = (env_lim[2] - env_space, env_lim[3] + env_space)

    matrix = np.zeros([np.int64(np.ceil((x_lim[1] - x_lim[0]) / resolution)),
                       np.int64(np.ceil((y_lim[1] - y_lim[0]) / resolution))])

    AgentID = 0
    agent= Agent_injector(AgentID, resolution, x_lim, y_lim, curr_drone_takeoff_pos, drone_name, matrix)