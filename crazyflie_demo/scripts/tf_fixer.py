#!/usr/bin/env python

import geometry_msgs.msg
import rospy
import tf2_ros


def handle_pose():
    r = rospy.Rate(40)
    prev_x = prev_y = prev_yaw = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    first_trans_cf = None
    first_trans_vrpn = None

    rospy.loginfo("waiting for initial transform from {} to world".format(tf_prefix))

    while (first_trans_cf == None):
        try:
            first_trans_cf = tfBuffer.lookup_transform('world', tf_prefix, rospy.Time(0))
        except:
            pass

    rospy.loginfo("waiting for initial transform from {} vrpn to world".format(tf_prefix))

    while (first_trans_vrpn == None):
        try:
            first_trans_vrpn = tfBuffer.lookup_transform('world', tf_prefix + '_vrpn', rospy.Time(0))
        except:
            pass

    init_x = first_trans_cf.transform.translation.x - first_trans_vrpn.transform.translation.x
    init_y = first_trans_cf.transform.translation.y - first_trans_vrpn.transform.translation.y

    rospy.loginfo("found initial transform for {}".format(tf_prefix))

    while not rospy.is_shutdown():

        trans = None

        try:  # if optitrack message exists
            trans = tfBuffer.lookup_transform('world', tf_prefix + '_vrpn', rospy.Time(0))

            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)

        except:
            rospy.loginfo("tf lookup -- {} not found".format(tf_prefix + '_vrpn'))

        if (trans != None):
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'world'
            t.child_frame_id = tf_prefix + '_fixed'
            t.transform.translation.x = -1 * trans.transform.translation.x - init_x
            t.transform.translation.y = -1 * trans.transform.translation.y - init_y
            t.transform.translation.z = trans.transform.translation.z

            # euler = euler_from_quaternion(q, axes='sxyz')
            #
            # roll = -1*euler[0]
            # pitch = -1*euler[1]
            # yaw = euler[2]
            #
            # q = tf_conversions.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)

        r.sleep()


if __name__ == '__main__':
    rospy.init_node('tf_fixer')
    tf_prefix = rospy.get_param("~tf_prefix")
    handle_pose()
