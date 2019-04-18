#!/usr/bin/env python
"""This is a simple Cj_injector for emulating"""
from math import radians

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def to_pose_stamped(x, y, z, roll, pitch, yaw):
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


# todo: motion controller is not working properly becayuse of Cj_injection's we need to make that work
if __name__ == '__main__':
    rospy.init_node("incjetor")

    prefix = rospy.get_param("~tf_prefix")
    Cj_injector_pub = rospy.Publisher('/' + prefix + "/Cj_injcetor", PoseStamped,
                                      queue_size=1)  # hover message publisher

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    time_delay = 2  # seconds
    while not rospy.is_shutdown():
        """Simple rectangle. """
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        [x, y, z, roll, pitch, yaw] = [1, 0, 0, 0, 0, 0]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        [x, y, z, roll, pitch, yaw] = [1, 1, 0, 0, 0, 0]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        [x, y, z, roll, pitch, yaw] = [0, 1, 0, 0, 0, 0]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        [x, y, z, roll, pitch, yaw] = [0, 0, 0, 0, 0, 6.28318530718]  # 2pi
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        [x, y, z, roll, pitch, yaw] = [0, 0, 0, 0, 0, 0]  # 0 pi
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)
