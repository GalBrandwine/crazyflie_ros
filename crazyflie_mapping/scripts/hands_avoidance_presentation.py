#!/usr/bin/env python


import sys
import termios
import time
import tty
from math import atan2
from threading import Thread

import crazyflie
import rospy
import tf2_geometry_msgs
# from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PoseStamped

# TODO: move all this code to be OOP, and fix this ugly thing with the threads. than add the thread solution to rother branches (or master)


speed = 0.25
initialZ = 1.0

global front, back, up, left, right, zrange, cj_injection_flag, cj_injection_message
front = back = up = left = right = zrange = 0.0
global ranges
ranges = []
cj_injection_flag = False
cj_injection_message = None


def Cj_injector(msg):
    global cj_injection_message, cj_injection_flag
    rospy.logdebug("Cj_recieved...")
    cj_injection_flag = True
    cj_injection_message = msg
    # rospy.loginfo(msg)


def check_direction(cj_injection_message):
    global listener, tfBuffer
    trans = None
    try:
        trans = tfBuffer.lookup_transform('world', prefix, rospy.Time(0))

    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix))
    if trans != None:
        cj_local_coord = PoseStamped()
        cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)
        rospy.loginfo(cj_local_coord)
        heading = atan2(cj_local_coord.pose.position.y, cj_local_coord.pose.position.x)
        rospy.loginfo(heading)


def avoid_collision():
    global msg, pub_hover
    rate = rospy.Rate(10)
    msg.header.frame_id = 'world'
    msg.yawrate = 0
    msg.zDistance = 0.4
    vx = -0.15
    vy = 0
    duration = 0.7
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.vx = vx
        msg.vy = vy
        now = rospy.get_time()
        if (now - start > duration):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.logdebug("sending...")
        rospy.logdebug(msg.vx)
        rospy.logdebug(now - start)
        pub_hover.publish(msg)
        rate.sleep()

    msg.header.seq += 1
    msg.vx = 0
    msg.vy = 0
    msg.header.stamp = rospy.Time.now()
    pub_hover.publish(msg)


def get_ranges(msg):
    global front, back, up, left, right, zrange, ranges
    front = msg.values[0] / 1000
    back = msg.values[1] / 1000
    up = msg.values[2] / 1000
    left = msg.values[3] / 1000
    right = msg.values[4] / 1000
    zrange = msg.values[5] / 1000
    ranges = [front, back, up, left, right, zrange]


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def keypress():
    global key
    key = getch()


def handler(cf_handler):
    r = rospy.Rate(5)
    time.sleep(1)
    cf_handler.takeoff(targetHeight=initialZ, duration=8.0)
    time.sleep(5.0)

    x, y, yaw = 0, 0, 0
    z = initialZ

    global key
    key = None
    global front, back, up, left, right, zrange
    dist_threshold = 0.1
    def_duration = 1.8

    try:
        rospy.loginfo("keyboard controller.")
        rospy.loginfo("press SPACE for emergency stop + land.")
        rospy.loginfo("press 's' for stop.")
        rospy.loginfo("press 'w' for forward.")
        rospy.loginfo("press 'x' for backward.")
        rospy.loginfo("press 'a' for left.")
        rospy.loginfo("press 'd' for right.")
        rospy.loginfo("press 'i' for up.")
        rospy.loginfo("press 'k' for down.")
        rospy.loginfo("press 'q','e' for yaw +-45 deg.")

        while not rospy.is_shutdown():
            if front > 0:
                if front < dist_threshold:
                    rospy.loginfo("forward collision avoidance")
                    cf_handler.goTo(goal=[-0.15, 0.0, 0.0], yaw=0, duration=def_duration / 100, relative=True)
                    time.sleep(def_duration)

                elif back < dist_threshold:
                    rospy.loginfo("back collision avoidance")
                    cf_handler.goTo(goal=[0.15, 0.0, 0.0], yaw=0, duration=def_duration / 100, relative=True)
                    time.sleep(def_duration)

                elif right < dist_threshold:
                    rospy.loginfo("right collision avoidance")
                    cf_handler.goTo(goal=[0.0, 0.15, 0.0], yaw=0, duration=def_duration / 100, relative=True)
                    time.sleep(def_duration)

                elif left < dist_threshold:
                    rospy.loginfo("left collision avoidance")
                    cf_handler.goTo(goal=[0.0, -0.15, 0.0], yaw=0, duration=def_duration / 100, relative=True)
                    time.sleep(def_duration)

                elif up < dist_threshold:
                    rospy.loginfo("top collision avoidance")
                    land_duration = z * 3
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    cf_handler.stop()
                    break

            # if key is not None:
            #s
            #     rozspy.loginfo("************* Key pressed is " + key.decode('utf-8'))
            #
            #     if key == ' ':
            #         # emergency land
            #         land_duration = z * 3
            #         cf_handler.land(targetHeight=0.0, duration=land_duration)
            #         time.sleep(land_duration - 0.5)
            #         cf_handler.stop()
            #         break
            #     elif key == 'w':
            #         # move forward
            #         cf_handler.goTo(goal=[0.25, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'x':
            #         # move backward
            #         cf_handler.goTo(goal=[-0.25, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'd':
            #         # move right
            #         cf_handler.goTo(goal=[0.0, -0.25, 0.0], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'a':
            #         # move left
            #         cf_handler.goTo(goal=[0.0, 0.25, 0.0], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'i':
            #         # move up
            #         cf_handler.goTo(goal=[0.0, 0.0, 0.05], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'k':
            #         # move down
            #         cf_handler.goTo(goal=[0.0, 0.0, -0.05], yaw=0, duration=def_duration, relative=True)
            #     elif key == 'q':
            #         # 45 degrees CW
            #         cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=1.5708, duration=def_duration + 1.0,
            #                 relative=True)  # slow down yaw rotation
            #     elif key == 'e':
            #         # 45 degrees CCW
            #         cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=-1.5708, duration=def_duration + 1.0,
            #                 relative=True)  # slow down yaw rotation
            #     # elif key == 's':
            #     # stop
            #
            #     key = None
            #     t2 = Thread(target=keypress, )
            #     t2.start()

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf_handler.stop()
        # break

    except Exception as e:
        cf_handler.stop()
        rospy.loginfo('*******keyboard input exception')
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('hands_avoidance_demonstration', log_level=rospy.DEBUG)

    prefix = rospy.get_param("~tf_prefix")
    rospy.Subscriber('/' + prefix + '/log_ranges', GenericLogData, get_ranges)

    cf = crazyflie.Crazyflie("/" + prefix, "world")

    rospy.wait_for_service("/" + prefix + '/update_params')
    rospy.loginfo("found update_params service")

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2)  # Use EKF

    cf.setParam("ctrlMel/kp_z", 0.8)  # reduce z wobble - default 1.25

    # reset kalman
    cf.setParam("kalman/initialX", 0)
    cf.setParam("kalman/initialY", 0)
    cf.setParam("kalman/initialZ", 0)
    cf.setParam("kalman/resetEstimation", 1)
    ########
    time.sleep(0.2)
    cf.setParam("kalman/resetEstimation", 0)
    time.sleep(0.5)
    cf.setParam("stabilizer/controller", 2)  # 2=Use mellinger controller
    time.sleep(0.2)

    rospy.loginfo("launching threads")
    t1 = Thread(target=handler, args=(cf,))
    t2 = Thread(target=keypress)
    t1.start()
    t2.start()
