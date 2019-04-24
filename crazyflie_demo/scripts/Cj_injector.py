#!/usr/bin/env python
"""This is a simple Cj_injector for emulating"""
from math import radians

import rospy
import tf
from geometry_msgs.msg import PoseStamped

"""
P*j - position vector (after filtering / conditioning . transformations) as ROS  structure - TransformStamped
S*j - distance sensor x,y,z point cloud 3*6 (after filtering / conditioning) as ROS  structure - PointCloud2 (global coordinates)

list of topics per drone:
    /cf4/cmd_full_state
    /cf4/cmd_hover
    /cf4/cmd_position
    /cf4/cmd_stop
    /cf4/cmd_vel
    /cf4/external_pose
    /cf4/external_position
    /cf4/imu
    /cf4/laserScan
    /cf4/log_pos                
    /cf4/log_ranges
    /cf4/log_rpy
    /cf4/odom
    /cf4/packets
    /cf4/point_cloud            <---- source for point cloud
    /cf4/rssi
    /tf                         <---- source for drone position/rotation 6dof



"""


class CrazyflieCjInjector:
    """Class for Cj injection per Cf


    """

    def __init__(self, tf_prefix):
        """Point_cloud contain a Point(x,y,z, index) per TOF sensors on CF_board.

        mean wile we have 4 TOF, sot len(point_cloud) = 4.
        index is the index of TOF sensor onboard.

        """
        # crazyflie name
        self.tf_prefix = tf_prefix

        self.point_cloud_last_timestamp = None
        self.point_cloud = []

        self.pos_last_timestamp = None
        self.pos = []

        # Init listeners
        self.Cj_injector_pub = rospy.Publisher('/' + prefix + "/Cj_injcetor", PoseStamped,
                                               queue_size=1)  # hover message publisher
        #
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # def point_cloud_parser(self, msg):
    #     """Each publication, we do a lookup for where the drone currently is, in x,y,z, r,p,y. """
    #     self.point_cloud_last_timestamp = msg.header
    #     self.point_cloud = pc2.read_points_list(msg, skip_nans=True)
    #
    #     try:
    #         trans = self.tfBuffer.lookup_transform('world', prefix, rospy.Time(0))
    #
    #         q = (trans.transform.rotation.x,
    #              trans.transform.rotation.y,
    #              trans.transform.rotation.z,
    #              trans.transform.rotation.w)
    #
    #         euler = euler_from_quaternion(q, axes='sxyz')
    #
    #         x = trans.transform.translation.x
    #         y = trans.transform.translation.y
    #         z = trans.transform.translation.z
    #         roll = euler[0]
    #         pitch = euler[1]
    #         yaw = euler[2]
    #
    #         self.pos = [x, y, z, roll, pitch, yaw]
    #
    #         # rospy.loginfo("pos: {}\n\n\n".format(self.pos))
    #
    #     except:
    #         rospy.logdebug("tf lookup -- {} not found".format(prefix))


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


# todo: motion controller is not working properly because of Cj_injection's we need to make that work

# todo change this to be an outside Node, that will receive N drones in a narray
if __name__ == '__main__':
    rospy.init_node("incjetor")

    prefix = rospy.get_param("~tf_prefix")
    Cj_injector_pub = rospy.Publisher('/' + prefix + "/Cj_injcetor", PoseStamped,
                                      queue_size=1)  # hover message publisher
    rospy.sleep(15)
    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    step_x = 0.5
    step_y = 0.5
    step_z = 0.3
    step_yaw = 90  # deg

    time_delay = 2  # seconds
    while not rospy.is_shutdown():
        x = 0.6
        y = 0.6
        z = 0.4
        """Simple rectangle. """
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)

        # [x, y, z, roll, pitch, yaw] = [step_x, 0, step_z, 0, 0, 0]
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
        #
        # [x, y, z, roll, pitch, yaw] = [step_x, step_y, step_z, 0, 0, 0]
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
        #
        # [x, y, z, roll, pitch, yaw] = [0, step_y, step_z, 0, 0, 0]
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
        #
        # [x, y, z, roll, pitch, yaw] = [0, 0, step_z, 0, 0, 0]
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
        #
        # [x, y, z, roll, pitch, yaw] = [0, 0, step_z, 0, 0, step_yaw]
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
        #
        # [x, y, z, roll, pitch, yaw] = [0, 0, step_z, 0, 0, 0]  # 0 pi
        # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        # Cj_injector_pub.publish(pose)
        # rospy.sleep(time_delay)
