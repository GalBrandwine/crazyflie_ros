#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

# initializes drone initial position 0,0,0
# launch drone to fixed height initialZ [m]
# collision avoidance #todo fix coordinate system currently works only without rotation
# command drone using KB in WORLD coordinatesz

import sys
import termios
import time
import tty
from math import atan2 , degrees, sqrt, pow, pi
import numpy as np
from threading import Thread

import crazyflie
import rospy
import tf2_geometry_msgs
import tf2_ros
# from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Hover  # imports for hover message
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# TODO: move all this shit into a class MotionController


speed = 0.25
initialZ = 0.35

global front, back, up, left, right, zrange, cj_injection_flag, cj_injection_message
front = back = up = left = right = zrange = 0.0
global ranges
ranges = []
cj_injection_flag = False
cj_injection_message = None


def Cj_injector(msg):
    global cj_injection_message, cj_injection_flag
    rospy.loginfo("Cj_received...")
    cj_injection_flag = True
    cj_injection_message = msg
    # rospy.loginfo(msg)

def check_direction():
    global listener, tfBuffer , cj_injection_message , heading
    trans = None
    try:
        trans = tfBuffer.lookup_transform(prefix ,'world', rospy.Time(0))
    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix))
    if trans != None:
        cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)
        rospy.loginfo(cj_local_coord)
        heading = atan2(cj_local_coord.pose.position.y, cj_local_coord.pose.position.x)
        distance= sqrt(pow(cj_local_coord.pose.position.x,2)+pow(cj_local_coord.pose.position.y,2))
        return heading , distance
    else:
        return False

# def avoid_collision():
#     global heading
#     global ranges
#     result = 0
#     if -3/4*pi < heading and heading > -1/4*pi:
#         if


def get_ranges(msg):
    global front, back, up, left, right, zrange, ranges, ranges_smoothed
    front = 0.8* front + 0.2 * msg.values[0] / 1000
    back = 0.8 * back + 0.2 * msg.values[1] / 1000
    up = msg.values[2] / 1000
    left = 0.8 * left + 0.2 * msg.values[3] / 1000
    right = 0.8 * right + 0.2 * msg.values[4] / 1000
    zrange = msg.values[5] / 1000
    ranges = [back, left, front, right, up, zrange]
    # ranges_np = np.asarray(ranges)
    # good_vals = np.nonzero(ranges_np)


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
    time.sleep(0.5)
    cf_handler.takeoff(targetHeight=initialZ, duration=4.0)
    time.sleep(5.0)

    x, y, yaw = 0, 0, 0
    z = initialZ

    global key
    key = None
    global cj_injection_message, cj_injection_flag
    global front, back, up, left, right, zrange
    dist_threshold = 0.15
    def_duration = 2.0

    try:
        rospy.loginfo("keyboard controller.\n")
        rospy.loginfo("press SPACE for emergency stop + land.\n")
        rospy.loginfo("press 's' for stop.\n")
        rospy.loginfo("press 'w' for forward.\n")
        rospy.loginfo("press 'x' for backward.\n")
        rospy.loginfo("press 'a' for left.\n")
        rospy.loginfo("press 'd' for right.\n")
        rospy.loginfo("press 'i' for up.\n")
        rospy.loginfo("press 'k' for down.\n")
        rospy.loginfo("press 'q','e' for yaw +-45 deg.\n")

        while not rospy.is_shutdown():

            if min(ranges) > 0:
                if front < dist_threshold:
                    rospy.loginfo("forward collision avoidance")
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.1, relative=True)
                    time.sleep(def_duration)

                elif back < dist_threshold:
                    rospy.loginfo("back collision avoidance")
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.1, relative=True)
                    time.sleep(def_duration)

                elif right < dist_threshold:
                    rospy.loginfo("right collision avoidance")
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.1, relative=True)
                    time.sleep(def_duration)

                elif left < dist_threshold:
                    rospy.loginfo("left collision avoidance")
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.1, relative=True)
                    time.sleep(def_duration)

                elif up < dist_threshold:
                    rospy.loginfo("top collision avoidance")
                    land_duration = z * 3
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    cf_handler.stop()
                    break

            # If Cj injection received:
            if cj_injection_flag is True:
                cj_injection_flag = False

                quaternion = (
                    cj_injection_message.pose.orientation.x,
                    cj_injection_message.pose.orientation.y,
                    cj_injection_message.pose.orientation.z,
                    cj_injection_message.pose.orientation.w)
                euler = euler_from_quaternion(quaternion)

                x = cj_injection_message.pose.position.x
                y = cj_injection_message.pose.position.y
                z = cj_injection_message.pose.position.z
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]

                direction,distance = degrees(check_direction())
                rospy.loginfo("DIRECTION", direction , "DISTANCE" , distance)
                obstacle_free=avoid_collision()
                if obstacle_free == True:
                    cf_handler.goTo(goal=[x, y, z], yaw=yaw, duration=def_duration, relative=False)
                else:
                    rospy.logwarn("cannot move - obstacle in the way")



            elif key is not None:

                rospy.loginfo("************* Key pressed is " + key.decode('utf-8'))

                if key == ' ':
                    # emergency land
                    land_duration = z * 3
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration - 0.5)
                    cf_handler.stop()
                    break
                elif key == 'w':
                    # move forward
                    cf_handler.goTo(goal=[0.3, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'x':
                    # move backward
                    cf_handler.goTo(goal=[-0.3, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'd':
                    # move right
                    cf_handler.goTo(goal=[0.0, -0.3, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'a':
                    # move left
                    cf_handler.goTo(goal=[0.0, 0.3, 0.0], yaw=0, duration=def_duration, relative=True)
                elif key == 'i':
                    # move up
                    cf_handler.goTo(goal=[0.0, 0.0, 0.05], yaw=0, duration=def_duration, relative=True)
                elif key == 'k':
                    # move down
                    cf_handler.goTo(goal=[0.0, 0.0, -0.05], yaw=0, duration=def_duration, relative=True)
                elif key == 'q':
                    # 45 degrees CW
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=1.5708, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation
                elif key == 'e':
                    # 45 degrees CCW
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=-1.5708, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation

                # todo add number for trajectorys
                # elif key == 's':
                # stop

                # todo FIX this stupid thing!
                key = None
                t2 = Thread(target=keypress, )
                t2.start()

            # print(" gospeed x: {}, y: {}, z: {} , yaw: {} \n".format( x, y, z ,yaw))
            # cf_handler.goSpeed(x, y, z, yaw)

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf_handler.stop()
        # break

    except Exception as e:
        cf_handler.stop()
        rospy.loginfo('******* exception thrown *********')
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('keyboard_controller')

    prefix = rospy.get_param("~tf_prefix")
    rospy.Subscriber('/' + prefix + '/log_ranges', GenericLogData, get_ranges)
    rospy.Subscriber('/' + prefix + '/Cj_injcetor', PoseStamped, Cj_injector)

    pub_hover = rospy.Publisher('/' + prefix + "/cmd_hover", Hover, queue_size=1)  # hover message publisher
    msg = Hover()
    msg.header.seq = 0

    tfBuffer = tf2_ros.Buffer()  # initialize tf buffer for transform lookup
    listener = tf2_ros.TransformListener(tfBuffer)

    cf = crazyflie.Crazyflie("/" + prefix, "world")

    rospy.wait_for_service("/" + prefix + '/update_params')
    rospy.loginfo("found update_params service")

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2)  # Use EKF

    cf.setParam("ctrlMel/kp_z", 0.8)  # reduce z wobble - default 1.25
    # cf.setParam("ctrlMel/ki_z", 0.06) #reduce z wobble - default 0.05
    # cf.setParam("ctrlMel/kd_z", 0.2) #reduce z wobble - default 0.4
    # cf.setParam("ctrlMel/i_range_z", 0.2) #reduce z wobble

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
