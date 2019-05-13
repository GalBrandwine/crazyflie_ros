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
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < time_delay):  # re-send point every X sec to reduce reception problems
            drone_injector.Cj_injector_pub.publish(pose)
            rospy.sleep(0.99)


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
        self.rotation_example_subscriber = rospy.Subscriber("/Cj_injection_rotation_example", Empty, self.launcher)

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
        step = 0.60  # m
        step_time = 1.0

        # simple straight line, then turn 360, then go back
        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_straight_line_360_go_back = [
            [step_time * 1, 0 * step, 8 * step, 0.35, 0],  # start in (0,180,0.35)
            [step_time * 2, 2 * step, 8 * step, 0.35, 0],  # go forward
            [step_time * 2, 4 * step, 8 * step, 0.35, 0],  # go forward
            [step_time * 2, 6 * step, 8 * step, 0.35, 0],  # go forward
            [step_time * 2, 6 * step, 8 * step, 0.35, 90],  # turn
            [step_time * 2, 6 * step, 8 * step, 0.35, 180],  # turn
            [step_time * 2, 6 * step, 8 * step, 0.35, 90],  # turn
            [step_time * 2, 6 * step, 8 * step, 0.35, 0],  # turn
            [step_time * 2, 4 * step, 8 * step, 0.35, 0],  # go back home
            [step_time * 2, 2 * step, 8 * step, 0.35, 0],  # go forward
            [step_time * 2, 0 * step, 8 * step, 0.35, 0],  # go land
            [step_time * 2, 0 * step, 8 * step, 0, 0],  # go land
        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_right_side = [[step_time* 0.5, 0 * step, 4.0 * step, 0.35, 0], #first point x=0 y=4
                                [step_time * 0.8, 0 * step, 3.8 * step, 0.35, 0],
                                [step_time * 4, 1 * step, 3.8 * step, 0.35, 0],
                                [step_time * 4, 2 * step, 3.8 * step, 0.35, 0],
                                [step_time * 4.8, 3.2 * step, 3.8 * step, 0.35, 0],
                                [step_time * 6, 3.2 * step, 5.3 * step, 0.35, 0],
                                [step_time * 2, 2.7 * step, 5.3 * step, 0.35, 0],
                                [step_time * 2.4, 3.3 * step, 5.3 * step, 0.35, 0],
                                [step_time * 6, 3.3 * step, 3.8 * step, 0.35, 0],
                                [step_time * 5.2, 4.6 * step, 3.8 * step, 0.35, 0],
                                [step_time * 4.8, 4.6 * step, 5 * step, 0.35, 0],
                                [step_time * 4.8, 4.6 * step, 6.2 * step, 0.35, 0],
                                [step_time * 4.8, 5.8 * step, 6.2 * step, 0.35, 0],
                                [step_time * 5.2, 7.1 * step, 6.2 * step, 0.35, 0],
                                [step_time * 4.8, 7.1 * step, 5 * step, 0.35, 0],
                                [step_time * 4.8, 7.1 * step, 3.8 * step, 0.35, 0],
                                [step_time * 1.6, 7.1 * step, 3.4 * step, 0.35, 0],
                                [step_time * 4, 8.1 * step, 3.4 * step, 0.35, 0],
                                [step_time * 4, 8.1 * step, 2.4 * step, 0.35, 0],
                                [step_time * 3.4, 8.1 * step, 3.25 * step, 0.35, 0],
                                [step_time * 4.8, 6.9 * step, 3.25 * step, 0.35, 0],
                                [step_time * 4.8, 5.7 * step, 3.25 * step, 0.35, 0],
                                [step_time * 4, 4.7 * step, 3.25 * step, 0.35, 0],
                                [step_time * 3.2, 4.7 * step, 2.45 * step, 0.35, 0],
                                [step_time * 3.2, 4.7 * step, 1.65 * step, 0.35, 0],
                                [step_time * 4, 3.7 * step, 1.65 * step, 0.35, 0],
                                [step_time * 4, 4.7 * step, 1.65 * step, 0.35, 0],
                                [step_time * 4, 4.7 * step, 2.65 * step, 0.35, 0],
                                [step_time * 4, 4.7 * step, 3.65 * step, 0.35, 0],
                                [step_time * 4.8, 3.5 * step, 3.65 * step, 0.35, 0],
                                [step_time * 4.8, 2.3 * step, 3.65 * step, 0.35, 0],
                                [step_time * 4.8, 1.1 * step, 3.65 * step, 0.35, 0],
                                [step_time * 4.4, 0 * step, 3.65 * step, 0.35, 0],

                                ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_left_side = [[step_time * 3, 0 * step, 4.5 * step, 0.35, 0],  # start in (0,300,0.35)
                               [step_time * 9, 5 * step, 4.5 * step, 0.35, 0],
                               [step_time * 4, 5 * step, 6.8 * step, 0.35, 0],
                               [step_time * 4, 7.5 * step, 6.8 * step, 0.35, 0],
                               [step_time * 4, 7.5 * step, 4 * step, 0.35, 0],
                               [step_time * 4, 6.5 * step, 4 * step, 0.35, 0],
                               [step_time * 4, 4.5 * step, 4 * step, 0.35, 0],
                               [step_time * 6, 0 * step, 4.5 * step, 0.35, 0],
                               [step_time * 2, 0 * step, 4 * step, 0, 0],
                               ]

        t1 = Thread(target=injector, args=(self.cj_injector_container[0], path_maze_right_side,))
        threads.append(t1)

        # t2 = Thread(target=injector, args=(self.cj_injector_container[1], path_maze_left_side,))
        # threads.append(t2)

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
