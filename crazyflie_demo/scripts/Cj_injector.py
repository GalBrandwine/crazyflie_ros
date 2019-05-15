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
        step =  0.3048  # one ft in meters
        step_time = 1.05

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
        path_maze_second_drone = [[step_time*7,0.25*step,6* step,0.35,0], #first point
                                [step_time * 1.575, 1.75 * step, 6 * step, 0.35, 0],
                                [step_time * 1.575, 3.25 * step, 6 * step, 0.35, 0],
                                [step_time * 1.05, 4.25 * step, 6 * step, 0.35, 0],
                                [step_time * 2.1, 4.25 * step, 8 * step, 0.35, 0],
                                [step_time * 1.05, 4.25 * step, 9 * step, 0.35, 45],
                                [step_time * 1.3125, 3 * step, 9 * step, 0.35, 45],
                                [step_time * 1.3125, 4.25 * step, 9 * step, 0.35, 45],
                                [step_time * 1.575, 4.25 * step, 7.5 * step, 0.35, 90],
                                [step_time * 1.575, 4.25 * step, 6 * step, 0.35, 90],
                                [step_time * 2.625, 6.75 * step, 6 * step, 0.35, 90],
                                [step_time * 2.205, 6.75 * step, 8.1 * step, 0.35, 135],
                                [step_time * 2.2575, 6.75 * step, 10.25 * step, 0.35, 135],
                                [step_time * 1.575, 8.25 * step, 10.25 * step, 0.35, 135],
                                [step_time * 1.8375, 10 * step, 10.25 * step, 0.35, 180],
                                [step_time * 1.575, 11.5 * step, 10.25 * step, 0.35, 180],
                                [step_time * 1.575, 11.5 * step, 8.75 * step, 0.35, 180],
                                [step_time * 1.575, 11.5 * step, 7.25 * step, 0.35, 180],
                                [step_time * 2.1, 11.5 * step, 5.25 * step, 0.35, 225],
                                [step_time * 1.575, 13 * step, 5.25 * step, 0.35, 225],
                                [step_time * 1.575, 13 * step, 3.75 * step, 0.35, 225],
                                [step_time * 1.8375, 13 * step, 5.5 * step, 0.35, 270],
                                [step_time * 2.1, 11 * step, 5.5 * step, 0.35, 270],
                                [step_time * 2.1, 9 * step, 5.5 * step, 0.35, 270],
                                [step_time * 2.1, 7 * step, 5.5 * step, 0.35, 315],
                                [step_time * 1.05, 6 * step, 5.5 * step, 0.35, 315],
                                [step_time * 1.575, 6 * step, 4 * step, 0.35, 315],
                                [step_time * 1.575, 6 * step, 2.5 * step, 0.35, 360],
                                [step_time * 2.1, 4 * step, 2.5 * step, 0.35, 360],
                                [step_time * 2.1, 6 * step, 2.5 * step, 0.35, 360],
                                [step_time * 1.8375, 6 * step, 4.25 * step, 0.35, 360],
                                [step_time * 1.8375, 6 * step, 6 * step, 0.35, 360],
                                [step_time * 2.1, 4 * step, 6 * step, 0.35, 360],
                                [step_time * 2.1, 2 * step, 6 * step, 0.35, 360],
                                [step_time * 1.8375, 0.25 * step, 6 * step, 0.35, 360],
                                ]

        #                       [----t----, ----x---, ---y---, --z--, yaw]
        path_maze_first_drone = [ [step_time*0.5,     1.0*step,6* step,0.35,0], #first drone x >0
                                [step_time * 2.1, 2.25 * step, 6 * step, 0.35, 0],
                                [step_time * 2.1, 4.25 * step, 6 * step, 0.35, 0],
                                [step_time * 2.3625, 6.5 * step, 6 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 8 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 10 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 4.5 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 2.5 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 2.5 * step, 14 * step, 0.35, 0],
                                [step_time * 1.8375, 4.25 * step, 14 * step, 0.35, 0],
                                [step_time * 2.1, 4.25 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 6.25 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 8.25 * step, 12 * step, 0.35, 0],
                                [step_time * 1.8375, 10 * step, 12 * step, 0.35, 0],
                                [step_time * 1.3125, 10 * step, 13.25 * step, 0.35, 0],
                                [step_time * 1.8375, 11.75 * step, 13.25 * step, 0.35, 0],
                                [step_time * 1.8375, 13.5 * step, 13.25 * step, 0.35, 0],
                                [step_time * 1.05, 14.5 * step, 13.25 * step, 0.35, 0],
                                [step_time * 1.3125, 14.5 * step, 12 * step, 0.35, 0],
                                [step_time * 1.8375, 12.75 * step, 12 * step, 0.35, 0],
                                [step_time * 1.8375, 11 * step, 12 * step, 0.35, 0],
                                [step_time * 1.8375, 9.25 * step, 12 * step, 0.35, 0],
                                [step_time * 1.8375, 7.5 * step, 12 * step, 0.35, 0],
                                [step_time * 1.05, 6.5 * step, 12 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 10 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 8 * step, 0.35, 0],
                                [step_time * 2.1, 6.5 * step, 6 * step, 0.35, 0],
                                [step_time * 2.1, 4.5 * step, 6 * step, 0.35, 0],
                                [step_time * 2.1, 2.5 * step, 6 * step, 0.35, 0],
                                [step_time * 2.3625, 0.25 * step, 6 * step, 0.35, 0],
                                ]

        t1 = Thread(target=injector, args=(self.cj_injector_container[0], path_maze_first_drone,))
        threads.append(t1)

        t2 = Thread(target=injector, args=(self.cj_injector_container[1], path_maze_second_drone,))
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
