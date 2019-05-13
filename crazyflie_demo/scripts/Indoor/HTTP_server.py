# !/usr/bin/python

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from time import sleep
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sensor_msgs.msg import CompressedImage


PORT_NUMBER = 8080

# This class will handle any incoming request from the browser
class RequestHandler(BaseHTTPRequestHandler):
    def __init__(self, matrix_size):
        # Initialize the matrix to be None
        self.matrix = None

        # Set the height and width of the matrix
        self.matrix_height = matrix_size[0]
        self.matrix_width = matrix_size[1]

        # A flag for car path plotting
        self.plot_car_flag = False

        # Set a subscriber for the occupancy grid. The occupancy grid is a part of the data in the matrix
        # (the occupancy grid contains the information about the free, ubknown and wall areas).
        self.grid_subscriber = rospy.Subscriber("/indoor/occupancy_grid_topic", OccupancyGrid,
                                        callback=self.grid_parser, callback_args="/indoor/occupancy_grid_topic")

        self.frame_sub = rospy.Subscriber('/indoor/vehicle_frame_topic', CompressedImage,
                                          callback=self.frame_parser)

    def do_GET(self):
        self.send_response(200)
        # Send one-time header for each connection
        self.send_header('Content-type', 'text/event-stream')
        self.send_header('x-matrix_height', '{}'.format(self.matrix_height))
        self.send_header('x-matrix_width', '{}'.format(self.matrix_width))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        # Start broadcasting data to all clients.
        self.start_broadcast()

    def start_broadcast(self):
        while True:
            flatten_matrix = None
            flatten_matrix = "test_data"
            message = "data: " + flatten_matrix + "\n\n"
            self.wfile.write(message)
            sleep(2)

    # Parse an occupancy grid message
    def grid_parser(self, msg):
        grid_height = int(msg.info.height / msg.info.resolution)
        grid_width = int(msg.info.width / msg.info.resolution)

        # Convert the 1D matrix in the message to 2D matrix
        self.matrix = np.array(msg.data).reshape((grid_height, grid_width))

        self.update_matrix_additional_data()

    # Update additional data: drones location, car path and location, etc.
    def update_matrix_additional_data(self):
        pass

    def frame_parser(self):
        pass




if __name__ == "__main__":
    rospy.init_node("http_server")
    try:
        server = HTTPServer(('10.0.2.15', PORT_NUMBER), RequestHandler)
        print 'Started HTTP server on port ', PORT_NUMBER

        server.serve_forever()

    except:
        server.socket.close()