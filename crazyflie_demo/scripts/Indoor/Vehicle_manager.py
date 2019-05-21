# !/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String
import numpy as np


class Vehicle_manager:
    def __init__(self):
        self.start_msg_sub = rospy.Subscriber('/indoor/start_vehicle_topic', String,
                                       callback=self.on_start_msg)

    def on_start_msg(self):
        pass


if __name__ == "__main__":
    rospy.init_node("vehicle_manager")

    vehicle_manager = Vehicle_manager()
