#!/usr/bin/env python

# sucscribes to log_ranges and publishes PointStanmed message in LC
# if transformation between 'drone' and 'world' exists, publishes PointStamped message in WC
# publishes laserscan in 'drone' frame, and converts it to PointCloud2 too.


import laser_geometry.laser_geometry as lg
import rospy
import tf2_ros
from math import pi, isinf
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from crazyflie_driver.msg import GenericLogData

global scan, seq
scan = None

global ranges_previous
ranges_previous = [0, 0, 0, 0]


def filter_scan():
    max_distance = 3.0  # maximum distance cutoff in meters
    outlier_threshold = 0.2  # maximum delta between two sequential readings, in meters
    inf = float('inf')
    global scan
    for i in range(len(scan.ranges)):
        if not isinf(scan.ranges[i]):  # distance filter
            if abs(scan.ranges[i]) > max_distance:
                scan.ranges[i] = inf
            # else:
            #     if abs(scan.ranges[i] - ranges_previous[i]) < outlier_threshold:
            #         ranges_previous[i] = scan.ranges[i] # remember current scan for next cycle
            #     else:
            #         scan.ranges[i] = inf #if delta between two sequential readings is too large , discard scan


def get_ranges(msg):
    global scan, seq  # pub_front, pub_back, pub_left, pub_right, ,
    global prev_ranges
    global tfBuffer, listener

    noise_threshold = 0.1

    front = msg.values[0] / 1000
    back = msg.values[1] / 1000
    left = msg.values[3] / 1000
    right = msg.values[4] / 1000
    z_range = msg.values[5] / 1000

    # rospy.logdebug("in publish scan: {}".format(msg))

    curr_ranges = [front, back, left, right]

    inf = float('inf')
    scan.ranges = [inf, inf, inf, inf]

    new_data = 0
    transform = None
    try:
        transform = tfBuffer.lookup_transform('world', rospy.get_param("~tf_prefix"), rospy.Time(0))
    except Exception as e:
        rospy.loginfo(e)

    if abs(curr_ranges[0] - prev_ranges[0]) > 0:
        scan.ranges[2] = curr_ranges[0]
        new_data += 1
        prev_ranges[0] = curr_ranges[0]

        ##publish point in local coordinates
        # point_front = PointStamped()
        # point_front.header.seq = seq
        # point_front.header.stamp = rospy.Time.now()
        # point_front.header.frame_id = rospy.get_param("~tf_prefix")
        # point_front.point.x = front
        # point_front.point.y = 0
        # point_front.point.z = 0
        # pub_front.publish(point_front)
        # prev_front = front
        #
        # ##publish another point in world coordinates
        # if (transform is not None):
        #     front_WC = tf2_geometry_msgs.do_transform_point(point_front, transform)
        #     front_WC.header.frame_id = "world"
        #     pub_front_WC.publish(front_WC)

    if abs(curr_ranges[1] - prev_ranges[1]) > 0:
        scan.ranges[0] = curr_ranges[1]
        new_data += 1
        prev_ranges[1] = curr_ranges[1]
        # point_back = PointStamped()
        # point_back.header.seq = seq
        # point_back.header.stamp = rospy.Time.now()
        # point_back.header.frame_id = rospy.get_param("~tf_prefix")
        # point_back.point.x = -1 * back
        # point_back.point.y = 0
        # point_back.point.z = 0
        # pub_back.publish(point_back)
        # prev_back = back
        #
        # ##publish another point in world coordinates
        # if (transform is not None):
        #     back_WC = tf2_geometry_msgs.do_transform_point(point_back, transform)
        #     back_WC.header.frame_id = "world"
        #     pub_back_WC.publish(back_WC)

    if abs(curr_ranges[2] - prev_ranges[2]) > 0:
        scan.ranges[3] = curr_ranges[2]
        new_data += 1
        prev_ranges[2] = curr_ranges[2]
        # point_left = PointStamped()
        # point_left.header.seq = seq
        # point_left.header.stamp = rospy.Time.now()
        # point_left.header.frame_id = rospy.get_param("~tf_prefix")
        # point_left.point.x = 0
        # point_left.point.y = left
        # point_left.point.z = 0
        # pub_left.publish(point_left)
        # prev_left = left
        #
        # ##publish another point in world coordinates
        # if (transform is not None):
        #     left_WC = tf2_geometry_msgs.do_transform_point(point_left, transform)
        #     left_WC.header.frame_id = "world"
        #     pub_left_WC.publish(left_WC)

    if abs(curr_ranges[3] - prev_ranges[3]) > 0:
        scan.ranges[1] = curr_ranges[3]
        new_data += 1
        prev_ranges[3] = curr_ranges[3]

        # point_right = PointStamped()
        # point_right.header.seq = seq
        # point_right.header.stamp = rospy.Time.now()
        # point_right.header.frame_id = rospy.get_param("~tf_prefix")
        # point_right.point.x = 0
        # point_right.point.y = -1 * right
        # point_right.point.z = 0
        # pub_right.publish(point_right)
        # prev_right = right
        #
        # ##publish another point in world coordinates
        # if (transform is not None):
        #     right_WC = tf2_geometry_msgs.do_transform_point(point_right, transform)
        #     right_WC.header.frame_id = "world"
        #     pub_right_WC.publish(right_WC)

    scan.header.stamp = rospy.Time.now()
    filter_scan()  # todo using scan as global variable. overwriting values. not good.
    scan_pub.publish(scan)  # publish LaserScan message in LOCAL (drone) coordinates

    if transform is not None and new_data > 0 and z_range >= 0.32:
        # Start publishing pointCloud only from 0.32 meters high
        pc2_msg_LC = lp.projectLaser(scan)  # convert the message of type LaserScan to a PointCloud2
        pc2_msg_WC = do_transform_cloud(pc2_msg_LC, transform)  # transform pointcloud to world coordinates
        pc2_pub.publish(pc2_msg_WC)  # publish pointcloud
        # rospy.loginfo(new_data)
    seq += 1


if __name__ == '__main__':
    rospy.init_node('publish_point', anonymous=False, log_level=rospy.DEBUG)

    # Crazyflie coordinate
    # pub_front = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/front', PointStamped, queue_size=1)
    # pub_back = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/back', PointStamped, queue_size=1)
    # pub_left = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/left', PointStamped, queue_size=1)
    # pub_right = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/right', PointStamped, queue_size=1)
    #
    # # World coordinate
    # pub_front_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/front_WC', PointStamped, queue_size=1)
    # pub_back_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/back_WC', PointStamped, queue_size=1)
    # pub_left_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/left_WC', PointStamped, queue_size=1)
    # pub_right_WC = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/points/right_WC', PointStamped, queue_size=1)

    scan_pub = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/laserScan', LaserScan, queue_size=2)
    pc2_pub = rospy.Publisher('/' + rospy.get_param("~tf_prefix") + '/point_cloud', PointCloud2, queue_size=1)

    rospy.Subscriber('/' + rospy.get_param("~tf_prefix") + '/log_ranges', GenericLogData, get_ranges)
    prev_ranges = [0, 0, 0, 0]
    seq = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    lp = lg.LaserProjection()

    scan = LaserScan()
    num_readings = 4
    laser_frequency = 10
    scan.header.frame_id = rospy.get_param("~tf_prefix")
    scan.angle_min = -pi
    scan.angle_max = pi
    scan.angle_increment = 2 * pi / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 3.5

    rospy.spin()
