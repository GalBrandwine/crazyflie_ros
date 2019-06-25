import rosbag, rospy, sys
import numpy as np

bagFileIn = sys.argv[1]
bagFileOut = sys.argv[2]

In = []
for topic, msg, t in rosbag.Bag(bagFileIn).read_messages():
    if topic=='/clock':
        In.append({'topic':topic,'msg':msg,'t':t})
    else:
        In.append({'topic':topic,'msg':msg,'t':msg.header.stamp})
   
with rosbag.Bag(bagFileOut, 'w') as outbag:
    for topic, msg, t in In:
            outbag.write(topic, msg, t)
