#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
from math import radians

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


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


class DroneCjInjector:
    """Class for Cj injection per drone. """

    def __init__(self, tf_prefix):
        """Init Cj_injector, and its publisher. """
        # crazyflie name
        self.tf_prefix = tf_prefix

        # Init listeners
        self.Cj_injector_pub = rospy.Publisher('/' + self.tf_prefix + "/Cj_injcetor", PoseStamped,
                                               queue_size=1)  # hover message publisher


class DroneInjector:
    """This class will subscribe to Mj topic, and send each drone it's next Cj. """

    def __init__(self, prefix_list, rate=15):
        self.cj_injector_container = []

        for i, prefix in enumerate(prefix_list):
            rospy.logdebug("\n\n******************* Initiating Cj_injector for: {}".format(prefix))

            drone = DroneCjInjector(prefix)
            self.cj_injector_container.append(drone)

        self.r = rate
        self.occ_map_subscriber_timestamp = None
        self.occ_map = None

        # Init listeners: Fill here when ready:
        # self.occ_map_subscriber = rospy.Subscriber("/" + prefix + "/point_cloud", ROS_TYPE_OCC_MAP,
        #                                self.point_cloud_parser)

        self.rotation_example_subscriber = rospy.Subscriber("/Cj_injection_rotation_example", Empty, self.launcher)

    def occ_map_subscriber(self, msg):
        """Each publication, we do a lookup for where the drone currently is, in x,y,z, r,p,y. """
        self.occ_map_subscriber_timestamp = msg.header
        self.occ_map = """Add here:Read the map from received 'msg'"""

        # try:
        #     trans = self.tfBuffer.lookup_transform('world', prefix, rospy.Time(0))
        #
        #     q = (trans.transform.rotation.x,
        #          trans.transform.rotation.y,
        #          trans.transform.rotation.z,
        #          trans.transform.rotation.w)
        #
        #     euler = euler_from_quaternion(q, axes='sxyz')
        #
        #     x = trans.transform.translation.x
        #     y = trans.transform.translation.y
        #     z = trans.transform.translation.z
        #     roll = euler[0]
        #     pitch = euler[1]
        #     yaw = euler[2]
        #
        #     self.pos = [x, y, z, roll, pitch, yaw]
        #
        #     # rospy.loginfo("pos: {}\n\n\n".format(self.pos))
        #
        # except:
        #     rospy.logdebug("tf lookup -- {} not found".format(prefix))

    def simple_rotation_example(self, time_delay=None):
        """Simple rotation example to make all drones rotate"""
        time_delay = time_delay if time_delay is not None else 2  # seconds
        # x = 0
        # y = 0
        # z = 0
        x = 1.2
        y = 1.2
        z = 0.35
        roll = 0
        pitch = 0
        yaw = 0

        step_x = 0.5
        step_y = 0.5
        step_z = 0.35
        step_yaw = 90  # deg
        for drone in self.cj_injector_container:
            pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
            drone.Cj_injector_pub.publish(pose)
            # rospy.sleep(time_delay)
            rospy.logdebug("******************* {} rotated!".format(drone))

    def simple_rectangel_example(self):
        """Make all drones in DroneInjector to fly in a rectangle.

        Very simple and stupid example for using the injcetor.

        """
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
            z = 0.3
            """first point in path. """
            for drone in self.cj_injector_container:
                pose = to_pose_stamped(x, y, step_z, roll, pitch, step_yaw)
                drone.Cj_injector_pub.publish(pose)
                rospy.sleep(time_delay)
                rospy.logdebug("******************* {} rotated!".format(drone))



            # [x, y, z, roll, pitch, yaw] = [step_x, 0, step_z, 0, 0, 0]
            # pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
            # Cj_injector_pub.publish(pose)
            # rospy.sleep(time_delay)

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

    def launcher(self, msg):
        """A callback for simple_examples. """
        self.simple_rotation_example()


# todo: motion controller is not working properly because of Cj_injection's we need to make that work

# todo change this to be an outside Node, that will receive N drones in a narray
if __name__ == '__main__':
    rospy.init_node("incjetor", log_level=rospy.DEBUG)
    Cj_injectors = []
    prefix_list_from_launch_file = rospy.get_param("~prefix_list")

    drone_container = DroneInjector(prefix_list_from_launch_file, 15)

    rospy.loginfo("******************* Publish /Cj_injection_rotation_example for starting the example")

    rospy.spin()
