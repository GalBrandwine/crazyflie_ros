import rosbag
import matplotlib.pyplot as pl
bagfilename='/home/rita/catkin_ws/2x2m_no_rotation.bag'
import sensor_msgs.point_cloud2 as pc2
import numpy as np


rb=rosbag.Bag(bagfilename)
# rbM=rb.read_messages("/cf4/log_pos")

posV_X=[]
posV_Y=[]
posV_Z=[]
with rosbag.Bag(bagfilename) as rb:
    for topic,msg,t in rb.read_messages():
        if topic=="/cf6/log_pos":
            # print(msg)
            posV_X.append(msg.values[0])
            posV_Y.append(msg.values[1])
            posV_Z.append(msg.values[2])

pl.plot(posV_X,posV_Y)

# sens0 = []
# sens1 = []
# sens2 = []
# sens3 = []
# with rosbag.Bag(bagfilename) as rb:
#     for topic,msg,t in rb.read_messages():
#         if topic=="/cf6/point_cloud":
#             # print(msg)
#             point_cloud = pc2.read_points_list(msg, skip_nans=True)
#             sens0.append(point_cloud[0])
#             sens1.append(point_cloud[1])
#             sens2.append(point_cloud[2])
#             sens3.append(point_cloud[3])


sens = []
with rosbag.Bag(bagfilename) as rb:
    for topic,msg,t in rb.read_messages():
        if topic=="/cf6/point_cloud":
            # print(msg)
            point_cloud = pc2.read_points_list(msg, skip_nans=True)
            for i in range(len(point_cloud)):
                point = point_cloud[i]
                sens.append([point.x , point.y, point.z])