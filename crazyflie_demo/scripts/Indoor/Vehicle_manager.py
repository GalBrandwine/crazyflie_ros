# !/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String
import numpy as np


class Vehicle_manager:
    def __init__(self):
        self.start_msg_sub = rospy.Subscriber('/indoor/start_vehicle_topic', String,
                                       callback=self.on_start_msg)
        self.received_grid_flag = False
        self.matrix = None

    def on_start_msg(self):
        self.grid_sub = rospy.Subscriber('/indoor/occupancy_grid_topic', OccupancyGrid,
                                              callback=self.grid_parser)

    def grid_parser(self, msg):
        if self.received_grid_flag == False:
            self.received_grid_flag = True

            grid_height = int(msg.info.height / msg.info.resolution)
            grid_width = int(msg.info.width / msg.info.resolution)
            self.matrix = np.array(msg.data).reshape((grid_height, grid_width))
            self.grid_sub.unregister()

            self.find_path()

    def find_path(self):
        pass


if __name__ == "__main__":
    rospy.init_node("vehicle_manager")

    vehicle_manager = Vehicle_manager()
