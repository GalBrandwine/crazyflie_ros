#!/usr/bin/env python
"""This is a simple Cj_injector for emulating. """
from threading import Thread

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from math import radians
from std_msgs.msg import Empty
from time import sleep


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


def injector(drone_injector, start_delay, path):
    """Simple threaded function for publishing point of a drone. """
    if start_delay > 0:
        sleep(start_delay)
        
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
            rospy.sleep(0.5)


class DroneInjector:
    """This class will subscribe to Mj topic, and send each drone it's next Cj. """

    def __init__(self, prefix_list, drone_delay_from_launch_file):
        self.cj_injector_container = []

        for i, prefix in enumerate(prefix_list):
            rospy.logdebug("\n\n******************* Initiating Cj_injector for: {}".format(prefix))

            drone = DroneCjInjector(prefix)
            self.cj_injector_container.append(drone)

        self.drone_delay_list = drone_delay_from_launch_file
        self.r = 15
        self.occ_map_subscriber_timestamp = None
        self.occ_map = None

        # Init listeners: Fill here when ready:
        self.rotation_example_subscriber = rospy.Subscriber("/Cj_injection_rotation_example", Empty, self.launcher)

    def simple_rotation_example(self, time_delay=None):
        """Simple rotation example to make all drones rotate"""

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
        x = 1.2
        y = 1.2
        z = 0.35
        roll = 0
        pitch = 0
        yaw = 0

        for drone in self.cj_injector_container:
            pose = to_pose_stamped(x, y, z, roll, pitch, yaw)
            drone.Cj_injector_pub.publish(pose)
            rospy.logdebug("******************* {} rotated!".format(drone))

    def simple_rectangel_example(self):
        """Make all drones in DroneInjector to fly in a rectangle.

        Very simple and stupid example for using the injcetor.

        path = [[duration,x,y,z,yaw], [], ...]

        """
        threads = []
        step = 0.3048  # one ft in meters
        step_time = 1.07

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame1 = [
            [step_time * 1, 1 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.3125, 2.25 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.3125, 3.5 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.3125, 3.5 * step, 7.75 * step, 0.3, 0],
            [step_time * 1.3125, 3.5 * step, 9 * step, 0.3, 0],
            [step_time * 1.8375, 1.75 * step, 9 * step, 0.3, 0],
            [step_time * 0.525, 1.75 * step, 9.5 * step, 0.3, 0],
            [step_time * 1.05, 2.75 * step, 9.5 * step, 0.3, 0],
            [step_time * 1.3125, 2.75 * step, 10.75 * step, 0.3, 0],
            [step_time * 1.575, 2.75 * step, 12.25 * step, 0.3, 0],
            [step_time * 0.7875, 2 * step, 12.25 * step, 0.3, 0],
            [step_time * 2.1, 2 * step, 14.25 * step, 0.3, 0],
            [step_time * 1.8375, 3.75 * step, 14.25 * step, 0.3, 0],
            [step_time * 2.1, 3.75 * step, 12.25 * step, 0.3, 0],
            [step_time * 1.575, 5.25 * step, 12.25 * step, 0.3, 0],
            [step_time * 1.575, 6.75 * step, 12.25 * step, 0.3, 0]

        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame2 = [
            [step_time * 1, 1 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 2.75 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 4.5 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 6.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.3125, 6.25 * step, 4 * step, 0.35, 0],
            [step_time * 1.05, 6.25 * step, 3 * step, 0.35, 0],
            [step_time * 1.575, 4.75 * step, 3 * step, 0.35, 0],
            [step_time * 1.575, 3.25 * step, 3 * step, 0.35, 0],
            [step_time * 1.05, 3.25 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 4.75 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 6.25 * step, 2 * step, 0.35, 0],
            [step_time * 1.575, 7.75 * step, 2 * step, 0.35, 0],
            [step_time * 1.3125, 9 * step, 2 * step, 0.35, 0],
            [step_time * 0.525, 9 * step, 2.5 * step, 0.35, 0],
            [step_time * 2.1, 7 * step, 2.5 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 4 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 5.5 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 7 * step, 0.35, 0],
            [step_time * 1.575, 7 * step, 8.5 * step, 0.35, 0],
            [step_time * 1.48492424049175, 8 * step, 9.5 * step, 0.35, 0],
            [step_time * 2.1, 8 * step, 11.5 * step, 0.35, 0]

        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame3 = [
            [step_time * 1, 0.25 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.8375, 2 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.8375, 3.75 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.8375, 5.5 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.575, 7 * step, 6.5 * step, 0.3, 0],
            [step_time * 1.05, 7 * step, 7.5 * step, 0.3, 0],
            [step_time * 1.8375, 7 * step, 9.25 * step, 0.3, 0],
            [step_time * 2.1, 9 * step, 9.25 * step, 0.3, 0],
            [step_time * 2.1, 11 * step, 9.25 * step, 0.3, 0],
            [step_time * 2.11634, 13 * step, 9 * step, 0.3, 0],
            [step_time * 1.1739, 14 * step, 9.5 * step, 0.3, 0],
            [step_time * 1.8375, 14 * step, 7.75 * step, 0.3, 0],
            [step_time * 1.05, 13 * step, 7.75 * step, 0.3, 0],
            [step_time * 1.3125, 11.75 * step, 7.75 * step, 0.3, 0],
            [step_time * 2.1, 11.75 * step, 5.75 * step, 0.3, 0],
            [step_time * 1.8375, 11.75 * step, 4 * step, 0.3, 0],
            [step_time * 1.05, 12.75 * step, 4 * step, 0.3, 0],
            [step_time * 1.575, 12.75 * step, 2.5 * step, 0.3, 0],
            [step_time * 1.05, 11.75 * step, 2.5 * step, 0.3, 0],
            [step_time * 1.575, 11.75 * step, 4 * step, 0.3, 0],
            [step_time * 1.575, 11.75 * step, 5.5 * step, 0.3, 0],
            [step_time * 1.575, 11.75 * step, 7 * step, 0.3, 0],
            [step_time * 1.575, 11.75 * step, 8.5 * step, 0.3, 0],
            [step_time * 1.575, 10.25 * step, 8.5 * step, 0.3, 0],
            [step_time * 1.48492424049175, 9.25 * step, 9.5 * step, 0.3, 0]

        ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_frame4 = [
            [step_time * 1, 0.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.575, 1.75 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.575, 3.25 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 5 * step, 5.25 * step, 0.35, 0],
            [step_time * 1.8375, 6.75 * step, 5.25 * step, 0.35, 0],
            [step_time * 2.1, 6.75 * step, 7.25 * step, 0.35, 0],
            [step_time * 2.1, 6.75 * step, 9.25 * step, 0.35, 0],
            [step_time * 1.575, 6.75 * step, 10.75 * step, 0.35, 0],
            [step_time * 1.575, 6.75 * step, 12.25 * step, 0.35, 0],
            [step_time * 0.742462120245875, 7.25 * step, 12.75 * step, 0.35, 0],
            [step_time * 1.05, 8.25 * step, 12.75 * step, 0.35, 0]

        ]

        t1 = Thread(target=injector, args=(self.cj_injector_container[0], self.drone_delay_list[0], path_maze_frame1,))
        threads.append(t1)

        t2 = Thread(target=injector, args=(self.cj_injector_container[1], self.drone_delay_list[1], path_maze_frame2,))
        threads.append(t2)

        t3 = Thread(target=injector, args=(self.cj_injector_container[2], self.drone_delay_list[2], path_maze_frame3,))
        threads.append(t3)

        t4 = Thread(target=injector, args=(self.cj_injector_container[3], self.drone_delay_list[3], path_maze_frame4,))
        threads.append(t4)

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
    rospy.init_node("incjetor")#log_level=rospy.DEBUG

    prefix_list_from_launch_file = rospy.get_param("~prefix_list")
    drone_delay_from_launch_file = rospy.get_param("~delay_list")
    drone_container = DroneInjector(prefix_list_from_launch_file, drone_delay_from_launch_file)

    rospy.loginfo("******************* Publish /Cj_injection_rotation_example for starting the example")

    rospy.spin()
