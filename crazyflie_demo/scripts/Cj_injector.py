#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
from threading import Thread

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from math import radians
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


def injector(drone_injector, path):
    """Simple threaded function for publishing point of a drone. """
    for point in path:
        time_delay = point[0]
        x = point[1]
        y = point[2]
        z = point[3]
        roll = 0
        pitch = 0
        yaw = point[4]
        pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
        drone_injector.Cj_injector_pub.publish(pose)
        rospy.sleep(time_delay)


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
        x = 0
        y = 0
        z = 0.35
        roll = 0
        pitch = 0
        yaw = 0

        step_x = 0.5
        step_y = 0.5
        step_z = 0.35
        step_yaw = 90  # deg
        for drone in self.cj_injector_container:
            pose = to_pose_stamped(x, y, z, roll, pitch, step_yaw)
            drone.Cj_injector_pub.publish(pose)
            # rospy.sleep(time_delay)
            rospy.logdebug("******************* {} rotated!".format(drone))

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

    def simple_rectangel_example(self, time_delay=None):
        """Make all drones in DroneInjector to fly in a rectangle.

        Very simple and stupid example for using the injcetor.

        path = [[duration,x,y,z,yaw], [], ...]

        """
        threads = []
        step = 0.30  # m
        #                       [t, ----x---, ---y---, --z--, yaw]
        path_maze_right_side = [[6, 0 * step, 6 * step, 0.35, 0],  # start in (0,180,0.35)
                                [2, 0 * step, 7 * step, 0.35, 0],  # proceed in  corridor
                                [2, 1 * step, 7 * step, 0.35, 0],  #
                                [2, 2 * step, 7 * step, 0.35, 0],  #
                                [2, 3 * step, 7 * step, 0.35, 0],  #
                                [2, 4 * step, 7 * step, 0.35, 0],  #
                                [2, 5 * step, 7 * step, 0.35, 0],  #
                                [2, 6 * step, 7 * step, 0.35, 0],  #
                                [2, 6.5 * step, 7 * step, 0.35, 0],  #
                                [4, 6.5 * step, 7 * step, 0.35, -90],  # rotate right in place
                                [2, 6.5 * step, 5 * step, 0.35, -90],  # enter into first room
                                [4, 6.5 * step, 5 * step, 0.35, -180],  # enter into first room
                                [2, 4 * step, 5 * step, 0.35, -180],    #
                                [4, 3.5 * step, 5 * step, 0.35, -90],   # scan
                                [4, 3.5 * step, 5 * step, 0.35, 0],  #
                                [4, 3.5 * step, 5 * step, 0.35, 90],    #
                                [4, 3.5 * step, 5 * step, 0.35, 0],  #
                                [4, 3.5 * step, 5 * step, 0.35, -90],   #
                                [2, 3.5 * step, 4 * step, 0.35, -90],  #
                                [2, 3.5 * step, 3 * step, 0.35, -90],  #
                                [2, 3.5 * step, 1 * step, 0.35, -90],  #
                                [4, 3.5 * step, 1 * step, 0.35, 0],  # turn 90 left
                                [2, 4 * step, 1 * step, 0.35, 0],  #
                                [2, 5 * step, 1 * step, 0.35, 0],  #
                                [2, 6.5 * step, 1 * step, 0.35, 0],  #
                                [4, 6.5 * step, 1 * step, 0.35, 90],  # turn 90 left
                                [2, 6.5 * step, 2 * step, 0.35, 90],  #
                                [2, 6.5 * step, 4 * step, 0.35, 90],  #
                                [2, 6.5 * step, 6 * step, 0.35, 90],  #
                                ]

        path_maze_left_side = [[2, 0 * step, 8 * step, 0.35, 0],  # start in (0,240,0.35)
                               [2, 0 * step, 7 * step, 0.35, 0],
                               [2, 1 * step, 7 * step, 0.35, 0],
                               [2, 2 * step, 7 * step, 0.35, 0],
                               [2, 3 * step, 7 * step, 0.35, 0],
                               [2, 4 * step, 7 * step, 0.35, 0],
                               [2, 5 * step, 7 * step, 0.35, 0],
                               [2, 6 * step, 7 * step, 0.35, 0],
                               [2, 7 * step, 7 * step, 0.35, 0],
                               [2, 8 * step, 7 * step, 0.35, 0],
                               [2, 9 * step, 7 * step, 0.35, 0],
                               [2, 9 * step, 8 * step, 0.35, 0],
                               [2, 9 * step, 9 * step, 0.35, 0],
                               [3, 9 * step, 10 * step, 0.35, 0],
                               [2, 9 * step, 11 * step, 0.35, 0],
                               [2, 9 * step, 12 * step, 0.35, 0],
                               [2, 8 * step, 12 * step, 0.35, 0],
                               [2, 7 * step, 12 * step, 0.35, 0],
                               [2, 6 * step, 12 * step, 0.35, 0],
                               [2, 5 * step, 12 * step, 0.35, 0],
                               [3.5, 5 * step, 12 * step, 0.35, 180],
                               ]

        path1 = [[2, 0.3, 0.6, 0.35, 0],
                 [2, 0.3, 0.9, 0.35, 0],
                 [2, 0.3, 1.2, 0.35, 0],
                 [2, 0.3, 1.5, 0.35, 0],
                 [2, 0.3, 1.8, 0.35, 0],
                 # [3, 0.3, 1.8, 0.35, 90],
                 [2, 0.6, 1.8, 0.35, 0],
                 [2, 0.9, 1.8, 0.35, 0],
                 [2, 1.2, 1.8, 0.35, 0],
                 [2, 1.5, 1.8, 0.35, 0],
                 [2, 1.8, 1.8, 0.35, 0],
                 [2, 2.1, 1.8, 0.35, 0],
                 # [3, 2.1, 1.8, 0.35, 0]
                 ]
        path2 = [[2, 2.1, 1.8, 0.35, 0],
                 [2, 2.1, 1.5, 0.35, 0],
                 [2, 2.1, 1.2, 0.35, 0],
                 [2, 2.1, 0.9, 0.35, 0],
                 [2, 2.1, 0.6, 0.35, 0],
                 # [3, 2.1, 0.6, 0.35, 90],
                 [2, 1.8, 0.6, 0.35, 0],
                 [2, 1.5, 0.6, 0.35, 0],
                 [2, 1.2, 0.6, 0.35, 0],
                 [2, 0.9, 0.6, 0.35, 0],
                 [2, 0.6, 0.6, 0.35, 0],
                 [2, 0.3, 0.6, 0.35, 0],

                 # [3, 0.3, 0.6, 0.35, 0]
                 ]

        # t1 = Thread(target=injector, args=(self.cj_injector_container[0], path1,))
        # threads.append(t1)
        # t2 = Thread(target=injector, args=(self.cj_injector_container[3], path2,))
        # threads.append(t2)

        t1 = Thread(target=injector, args=(self.cj_injector_container[0], path_maze_right_side,))
        threads.append(t1)

        t2 = Thread(target=injector, args=(self.cj_injector_container[1], path_maze_left_side,))
        threads.append(t2)

        # start all threads.
        for t in threads:
            t.start()

        # stop workers
        for t in threads:
            t.join()

        rospy.loginfo("********************************* Done ***************************")

    def launcher(self, msg):
        """A callback for simple_examples. """
        # self.simple_rotation_example()
        self.simple_rectangel_example()


if __name__ == '__main__':
    rospy.init_node("incjetor", log_level=rospy.DEBUG)
    Cj_injectors = []
    prefix_list_from_launch_file = rospy.get_param("~prefix_list")

    drone_container = DroneInjector(prefix_list_from_launch_file, 15)

    rospy.loginfo("******************* Publish /Cj_injection_rotation_example for starting the example")

    rospy.spin()
