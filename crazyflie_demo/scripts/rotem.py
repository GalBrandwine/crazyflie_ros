#!/usr/bin/env python
"""This scrips capture's all "ross" data of a specific CF, and contain it in a pythonize object, for later use. """

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion

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


class CrazyflieFlightData:
    """A class for containing a crazyflie flight data.

    than later on, it will be used for rotem's alg'.
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
        self.pc_sub = rospy.Subscriber("/" + prefix + "/point_cloud", PointCloud2,
                                       self.point_cloud_parser)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def point_cloud_parser(self, msg):
        """Each publication, we do a lookup for where the drone currently is, in x,y,z, r,p,y. """
        self.point_cloud_last_timestamp = msg.header
        self.point_cloud = pc2.read_points_list(msg, skip_nans=True)



        try:
            trans = self.tfBuffer.lookup_transform('world', prefix, rospy.Time(0))

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

            self.pos = [x, y, z, roll, pitch, yaw]

            #rospy.loginfo("pos: {}\n\n\n".format(self.pos))

        except:
            rospy.logdebug("tf lookup -- {} not found".format(prefix))


if __name__ == '__main__':
    rospy.init_node("drone_data_parser")

    # get cf name
    prefix = rospy.get_param("~tf_prefix")
    CrazyflieFlightData(prefix)

    rospy.spin()
