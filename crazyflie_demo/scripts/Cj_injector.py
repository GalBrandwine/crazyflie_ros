#!/usr/bin/env python
"""This is a simple Cj_injector for emulating"""
from math import radians

import geometry_msgs.msg
import rospy
import tf2_ros
import tf_conversions

if __name__ == '__main__':
    rospy.init_node("mynode")

    # prefix = rospy.get_param("~tf_prefix")
    prefix = "drone"

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    while True:
        """Simple rectangle. """
        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = rospy.get_param("~tf_prefix") + '_takeoff'
        t.header.frame_id = 'world'
        # t.child_frame_id = rospy.get_param("~tf_prefix")
        t.child_frame_id = prefix
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        rospy.sleep(1)

        [x, y, z, roll, pitch, yaw] = [1, 0, 0, 0, 0, 0]

        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = rospy.get_param("~tf_prefix") + '_takeoff'
        t.header.frame_id = 'world'
        # t.child_frame_id = rospy.get_param("~tf_prefix")
        t.child_frame_id = prefix
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        rospy.sleep(1)

        [x, y, z, roll, pitch, yaw] = [1, 1, 0, 0, 0, 0]

        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = rospy.get_param("~tf_prefix") + '_takeoff'
        t.header.frame_id = 'world'
        # t.child_frame_id = rospy.get_param("~tf_prefix")
        t.child_frame_id = prefix
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        rospy.sleep(1)

        [x, y, z, roll, pitch, yaw] = [0, 1, 0, 0, 0, 0]

        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = rospy.get_param("~tf_prefix") + '_takeoff'
        t.header.frame_id = 'world'
        # t.child_frame_id = rospy.get_param("~tf_prefix")
        t.child_frame_id = prefix
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        rospy.sleep(1)

        [x, y, z, roll, pitch, yaw] = [0, 0, 0, 0, 0, 6.28318530718]  # 2pi

        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = rospy.get_param("~tf_prefix") + '_takeoff'
        t.header.frame_id = 'world'
        # t.child_frame_id = rospy.get_param("~tf_prefix")
        t.child_frame_id = prefix
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        rospy.sleep(1)

        [x, y, z, roll, pitch, yaw] = [0, 0, 0, 0, 0, 0]  # 0 pi
    # goal.header.seq = 1
    # goal.header.stamp = rospy.Time.now()
    # goal.header.frame_id = "map"
    #
    # goal.pose.position.x = 1.0
    # goal.pose.position.y = 2.0
    # goal.pose.position.z = 0.0
    #
    # goal.pose.orientation.x = 0.0
    # goal.pose.orientation.y = 0.0
    # goal.pose.orientation.z = 0.0
    # goal.pose.orientation.w = 1.0
    #
    # rospy.sleep(1)
    # goal_publisher.publish(goal)

    # rospy.spin()
