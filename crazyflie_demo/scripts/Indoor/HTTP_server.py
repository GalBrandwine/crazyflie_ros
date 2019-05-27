#!/usr/bin/python

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from time import sleep
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
import json
import tf2_ros
from tf.transformations import euler_from_quaternion
import numpy as np
from Grid import m_to_cm

PORT_NUMBER = 8080

# This class will handle any incoming request from the browser
class RequestHandler(BaseHTTPRequestHandler):
    def initialize(self):
        env_lim = rospy.get_param("~env_lim")
        env_space = rospy.get_param("~env_space")

        exec("env_lim = {}".format(env_lim))

        self.x_lim = (env_lim[0] - env_space, env_lim[1] + env_space)
        self.y_lim = (env_lim[2] - env_space, env_lim[3] + env_space)
        
        # Initialize the matrix to be None
        self.matrix = None

        # Set the height and width of the matrix to be None
        self.matrix_height = None
        self.matrix_width = None

        self.grid_resolution = rospy.get_param("~resolution")

        # A flag for car path plotting
        self.plot_car_flag = False

        # Initial vehicle position
        self.vehicle_start_pos = rospy.get_param("~vehicle_start_pos")
        exec ("self.vehicle_start_pos = {}".format(self.vehicle_start_pos))
        self.vehicle_pos = np.multiply(self.vehicle_start_pos, m_to_cm)
        self.vehicle_pos_history = []

        # Initialize listener to drone locations
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.drone_pos_dict = {}
        self.nDrones = rospy.get_param('~nDrones')
        for iDrone in range(self.nDrones):
            curr_drone_name = rospy.get_param("~drone_name_{}".format(iDrone))
            self.drone_pos_dict[curr_drone_name] = None

        # Load recorded vehicle path
        # self.vehicle_path = np.loadtxt(rospy.get_param('~vehicle_recorded_path'))

        # Set a subscriber for the occupancy grid. The occupancy grid is a part of the data in the matrix
        # (the occupancy grid contains the information about the free, ubknown and wall areas).
        self.grid_sub = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                         callback=self.grid_parser)

        self.vehicle_pos_sub = rospy.Subscriber("/slam_out_pose", PoseStamped,
                                         callback=self.vehicle_pos_parser)

        # self.frame_sub = rospy.Subscriber('/camera/infra1/image_rect_raw/compressed', CompressedImage,
        #                                   callback=self.frame_parser)

    def do_GET(self):
        
        self.initialize()
        
        while self.matrix_width is None and self.matrix_height is None:
            sleep(1)
            
        self.send_response(200)

        print "connection established"

        # Send one-time grid size for each connection
        self.broadcast_grid_size()

        # Start broadcasting data to all clients.
        self.start_broadcast_grid()

    def broadcast_grid_size(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-type', 'text/event-stream')
        self.end_headers()

        grid_size = dict(rows=self.matrix_height, cols=self.matrix_width)
        grid_size_json = json.dumps(grid_size)
        res_str = "event: size\n"
        res_str += "data: {}\n\n".format(grid_size_json)
        # print res_str
        self.wfile.write(res_str.encode("utf-8"))

    def start_broadcast_grid(self):
        while True:
            #  Reflect the matrix
            reflected_matrix = np.transpose(self.matrix)
            json_res = json.dumps(reflected_matrix.tolist())
            res_str = "event: grid\n"
            res_str += "data: {}\n\n".format(json_res)
            self.wfile.write(res_str.encode("utf-8"))

            sleep(0.5)

    # Parse an occupancy grid message
    def grid_parser(self, msg):
        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)
        
        if self.matrix_height is None:
            self.matrix_height = grid_height
        if self.matrix_width is None:
            self.matrix_width = grid_width

        # Convert the 1D matrix in the message to 2D matrix
        self.matrix = np.array(msg.data).reshape((grid_width, grid_height))

        self.update_matrix_additional_data()

    def vehicle_pos_parser(self, msg):
        self.vehicle_pos = (msg.pose.position.x, msg.pose.position.y)
        self.vehicle_pos = np.multiply(np.add(self.vehicle_pos, self.vehicle_start_pos), m_to_cm)
        vehicle_pos_ij = list(np.floor(np.divide(np.subtract(self.vehicle_pos, [self.x_lim[0], self.y_lim[0]]), self.grid_resolution)))
        self.vehicle_pos_history.append(vehicle_pos_ij)

    # Update additional data: drones location, car path and location, etc.
    def update_matrix_additional_data(self):
        # for i in range(len(self.vehicle_pos_history)):
        #     if self.vehicle_pos_history[i][0] >= 0 and self.vehicle_pos_history[i][0] < np.shape(self.matrix[0]) and \
        #             self.vehicle_pos_history[i][1] >= 0 and self.vehicle_pos_history[i][1] < np.shape(self.matrix[1]):
        #         self.matrix[self.vehicle_pos_history[i][0], self.vehicle_pos_history[i][1]] = 5

        self.update_drone_positions()
        for iDrone in range(self.nDrones):
            drone_name = self.drone_pos_dict.keys()[iDrone]
            if self.drone_pos_dict[drone_name] is None:
                continue
            drone_pos_xy = self.drone_pos_dict[drone_name][:2]
            drone_pos_ij = list(np.floor(np.divide(np.subtract(drone_pos_xy, [self.x_lim[0], self.y_lim[0]]), self.grid_resolution)))
            self.matrix[int(drone_pos_ij[0]), int(drone_pos_ij[1])] = 3

        # vehicle_pos_ij = list(np.floor(np.divide(np.subtract(self.vehicle_pos, [self.x_lim[0], self.y_lim[0]]), self.grid_resolution)))
        # if vehicle_pos_ij[0] >= 0 and vehicle_pos_ij[0] < np.shape(self.matrix[0]) and vehicle_pos_ij[1] >= 0 and vehicle_pos_ij[1] < np.shape(self.matrix[1]):
        #     self.matrix[int(vehicle_pos_ij[0]), int(vehicle_pos_ij[1])] = 4

    def update_drone_positions(self):
        for iDrone in range(len(self.drone_pos_dict)):
            drone_id = self.drone_pos_dict.keys()[iDrone]
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

                # Store drone position and convert it from [m] to [cm]
                self.drone_pos_dict[drone_id] = [x * m_to_cm, y * m_to_cm, z * m_to_cm, roll, pitch, yaw]

            except Exception as e:
                rospy.logdebug("tf lookup -- {} not found".format(drone_id))
            # except Exception as e:
            #     rospy.loginfo(e)

    def frame_parser(self, msg):
        pass

if __name__ == "__main__":
    rospy.init_node("http_server")
    try:
        server = HTTPServer(('localhost', PORT_NUMBER), RequestHandler)
        # server = HTTPServer(('192.168.1.113', PORT_NUMBER), RequestHandler)
        print 'Started HTTP server on port ', PORT_NUMBER

        server.serve_forever()

    except:
        server.socket.close()

