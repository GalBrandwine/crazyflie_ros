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
from math import atan2, sqrt, pow

import crazyflie
import rospy
import tf2_geometry_msgs
import tf2_ros
# from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# TODO: move all this shit into a class MotionController


speed = 0.25
initialZ = 0.35

global front, back, up, left, right, zrange, cj_injection_flag, cj_injection_message
front = back = up = left = right = zrange = 0.0

global kb_x, kb_y, kb_z, kb_yaw
kb_x = kb_y = kb_z = kb_yaw = 0

global ranges
ranges = []
cj_injection_flag = False
cj_injection_message = None

keyboard_flag = False


def twist_callback(msg):
    global kb_x, kb_y, kb_z, kb_yaw, keyboard_flag
    def_duration = 2.0
    land_duration = 1.5

    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

    kb_x = msg.linear.x
    kb_y = msg.linear.y
    kb_z = msg.linear.z
    kb_yaw = msg.angular.z

    keyboard_flag = True


def Cj_injector(msg):
    global cj_injection_message, cj_injection_flag
    rospy.logdebug("Cj_recieved...")
    cj_injection_flag = True
    cj_injection_message = msg
    # rospy.loginfo(msg)


def check_direction():
    global listener, tfBuffer, cj_injection_message

    speed = 0.20  # default speed m/s
    rot_speed = 2.0  # default rot speed sec/radian
    min_duration = 1  # minimum time [sec] for single trajectory

    trans = None
    try:
        trans = tfBuffer.lookup_transform(prefix, 'world', rospy.Time(0))
    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix))
    if trans != None:
        cj_local_coord = PoseStamped()
        cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)
        rospy.loginfo(cj_local_coord)
        heading = atan2(cj_local_coord.pose.position.y, cj_local_coord.pose.position.x)
        rospy.loginfo(heading)

        distance = sqrt(pow(cj_local_coord.pose.position.x, 2) + pow(cj_local_coord.pose.position.y, 2))
        duration = distance / speed  # #calculate required time for this motion
        if duration < min_duration: duration = min_duration

        q = (cj_local_coord.pose.orientation.x,
             cj_local_coord.pose.orientation.y,
             cj_local_coord.pose.orientation.z,
             cj_local_coord.pose.orientation.w)

        euler = euler_from_quaternion(q, axes='sxyz')

        rotation_yaw = euler[2]

        duration_yaw = rotation_yaw * rot_speed

        if duration_yaw > duration:
            duration = duration_yaw

        return [heading, duration]
    else:
        return False


# def avoid_collision():
#     global heading
#     global ranges
#     result = 0
#     if -3/4*pi < heading and heading > -1/4*pi:
#         if


def get_ranges(msg):
    global front, back, up, left, right, zrange, ranges
    weight_old = 0.65  # the weight given to old values in filter 0-1
    weight_new = 1.0 - weight_old
    front = weight_old * front + weight_new * msg.values[0] / 1000  # low-pass filter on range inputs used for collision
    back = weight_old * back + weight_new * msg.values[1] / 1000
    up = weight_old * up + weight_new * msg.values[2] / 1000
    left = weight_old * left + weight_new * msg.values[3] / 1000
    right = weight_old * right + weight_new * msg.values[4] / 1000
    # zrange = msg.values[5] / 1000
    ranges = [back, left, front, right, up, zrange]


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def get_xyz_yaw(cj_injection_message):
    """Transform Cj injection into drone coordinates"""
    trans = None
    try:
        trans = tfBuffer.lookup_transform(prefix + '_takeoff', 'world', rospy.Time(0))
    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix + '_takeoff'))
    if trans is not None:
        cj_local_coord = PoseStamped()
        cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)

        quaternion = (
            cj_local_coord.pose.orientation.x,
            cj_local_coord.pose.orientation.y,
            cj_local_coord.pose.orientation.z,
            cj_local_coord.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)

        x = cj_local_coord.pose.position.x
        y = cj_local_coord.pose.position.y
        z = cj_local_coord.pose.position.z
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return [x, y, z, yaw]
    else:
        return False


def handler(cf_handler):
    r = rospy.Rate(5)
    time.sleep(0.5)
    cf_handler.takeoff(targetHeight=initialZ, duration=4.0)
    time.sleep(5.0)

    x, y, yaw = 0, 0, 0
    z = initialZ

    global keyboard_flag
    global cj_injection_message, cj_injection_flag
    global front, back, up, left, right, zrange
    dist_threshold = 0.15
    def_duration = 2.0

    # There's 2 levels of collision protections:
    #   Low level:
    #       The threshold for protecting drone before collision with walls
    # High level:
    #       avoid_collision(), a function that will be called every time theres a
    #       Cj_injection and before that Cj order will go to drone,
    #       we will check if theres a futoristic collision within that path.

    dist_threshold = 0.25
    def_duration = 1.8
    land_duration = 1.5

    try:

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
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    cf_handler.stop()
                    break

            if keyboard_flag == True:
                keyboard_flag = False
                if kb_x > 0:
                    cf_handler.goTo(goal=[0.25, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif kb_x < 0:
                    cf_handler.goTo(goal=[-0.25, 0.0, 0.0], yaw=0, duration=def_duration, relative=True)
                elif kb_yaw < 0:
                    cf_handler.goTo(goal=[0.0, -0.25, 0.0], yaw=0, duration=def_duration, relative=True)
                elif kb_yaw > 0:
                    cf_handler.goTo(goal=[0.0, 0.25, 0.0], yaw=0, duration=def_duration, relative=True)
                elif kb_y > 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=1.5708, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation
                elif kb_y < 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=-1.5708, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation
                elif kb_x == 0 and kb_y == 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.6, relative=True)

                if kb_z != 0:
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration - 0.5)
                    cf_handler.stop()

            # If Cj injection received:
            if cj_injection_flag is True:
                cj_injection_flag = False
                # get Cj_injection in drone coordinates
                [x, y, z, yaw] = get_xyz_yaw(cj_injection_message)
                [direction, duration] = check_direction()

                rospy.loginfo("Cj direction is ".format(direction))
                rospy.loginfo("Cj duration is ".format(duration))

                # obstacle_free=avoid_collision()
                # if obstacle_free == True:
                cf_handler.goTo(goal=[x, y, z], yaw=yaw, duration=duration, relative=False)
                # else:
                #     rospy.logwarn("cannot move - obstacle in the way")

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf_handler.stop()
        # break

    except Exception as e:
        cf_handler.stop()
        rospy.loginfo('*******keyboard input exception')
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('keyboard_controller', )  # log_level=rospy.DEBUG

    prefix = rospy.get_param("~tf_prefix")
    rospy.Subscriber('/' + prefix + '/log_ranges', GenericLogData, get_ranges)
    rospy.Subscriber('/' + prefix + '/Cj_injcetor', PoseStamped, Cj_injector)

    # pub_hover = rospy.Publisher('/' + prefix + "/cmd_hover", Hover, queue_size=1)  # hover message publisher
    # msg = Hover()
    # msg.header.seq = 0

    rospy.Subscriber("/cmd_vel", Twist, twist_callback)

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

    handler(cf)
    # t1 = Thread(target=handler, args=(cf,))
    # # t2 = Thread(target=keypress)
    # t1.start()
    # # t2.start()
