#!/usr/bin/env python
# source - https://github.com/whoenig/crazyflie_ros/commit/b048c1f2fd3ee34f899fa0e2f6c58a4885a39405#diff-970be3522034ff436332d391db26982a

# initializes drone initial position 0,0,0
# launch drone to fixed height initialZ [m]
# collision avoidance #todo fix coordinate system currently works only without rotation
# command drone using KB in WORLD coordinatesz

import time

import rospy
import tf2_geometry_msgs
import tf2_ros
# from crazyflie_driver.msg import Hover
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pow
from math import pi, sin, cos
from tf.transformations import euler_from_quaternion
from math import radians
import crazyflie
# from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData

# TODO: move all this shit into a class MotionController


speed = 0.25
initialZ = 0.35

global front, back, up, left, right, zrange, cj_injection_flag, cj_injection_message
front = back = up = left = right = zrange = 0.0

global kb_x, kb_y, kb_z, kb_yaw
kb_x = kb_y = kb_z = kb_yaw = 0

global listen_to_keyboard
global ranges
ranges = [0, 0, 0, 0]
cj_injection_flag = False
cj_injection_message = None

listen_to_keyboard = False
keyboard_flag = False


def twist_callback(msg):
    global kb_x, kb_y, kb_z, kb_yaw, keyboard_flag, listen_to_keyboard

    if listen_to_keyboard is 1:
        def_duration = 2.0
        land_duration = 1.5

        # rospy.loginfo("Received a /cmd_vel message!")
        # rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        # rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

        kb_x = msg.linear.x
        kb_y = msg.linear.y
        kb_z = msg.linear.z
        kb_yaw = msg.angular.z

        keyboard_flag = True


def Cj_injector(msg):
    global cj_injection_message, cj_injection_flag
    # rospy.logdebug("Cj_recieved...")
    cj_injection_flag = True
    cj_injection_message = msg
    # rospy.loginfo(msg)


def check_direction():
    global listener, tfBuffer, cj_injection_message

    speed = 0.20  # default speed m/s
    rot_speed = 0.5  # default rot speed sec/radian
    min_duration = 2.0  # minimum time [sec] for single trajectory
    duration = default_duration = 2  # sec
    trans = None
    try:
        trans = tfBuffer.lookup_transform(prefix, 'world', rospy.Time(0))
    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix))
    if trans != None:
        cj_local_coord = PoseStamped()
        cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)
        # rospy.loginfo(cj_local_coord)
        heading = atan2(cj_local_coord.pose.position.y, cj_local_coord.pose.position.x)
        # rospy.loginfo(heading)

        distance = sqrt(pow(cj_local_coord.pose.position.x, 2) + pow(cj_local_coord.pose.position.y, 2))
        duration = distance / speed  # #calculate required time for this motion
        # rospy.logdebug("in check_direction: before if: [{},{}]".format(heading, duration))
        if duration < min_duration:
            duration = min_duration

        q = (cj_local_coord.pose.orientation.x,
             cj_local_coord.pose.orientation.y,
             cj_local_coord.pose.orientation.z,
             cj_local_coord.pose.orientation.w)

        euler = euler_from_quaternion(q, axes='sxyz')

        rotation_yaw = abs(euler[2])

        duration_yaw = rotation_yaw / rot_speed
        # rospy.logdebug("in check_direction: duration_yaw: {}".format(duration_yaw))
        # rospy.logdebug("in check_direction: duration: {}".format(duration))
        if duration_yaw > duration:
            duration = duration_yaw

        # rospy.logdebug("in check_direction. returning: [{},{}]".format(heading, duration))
        return [heading, duration]
    else:
        rospy.logerr("in check_direction: transform is None")
        return [0, duration]


def collision_direction_wc(collision_sensor_angle):
    evade_distance = 0.07  # distance to go opposite direction of wall - meters

    global prefix
    trans = None
    try:
        trans = tfBuffer.lookup_transform(prefix, 'world', rospy.Time(0))
    except:
        rospy.logwarn("tf lookup -- {} not found".format(prefix))
    if trans != None:
        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)

        euler = euler_from_quaternion(q, axes='sxyz')
        yaw = euler[2]

        collision_angle_wc = collision_sensor_angle + yaw
        if collision_angle_wc < -2 * pi:
            collision_angle_wc += 2 * pi

        if collision_angle_wc > 2 * pi:
            collision_angle_wc -= 2 * pi

        y = evade_distance * sin(collision_angle_wc)
        x = evade_distance * cos(collision_angle_wc)
        avoid_goal = [-x, y]
        # rospy.loginfo('collision at angle LC')
        # rospy.loginfo(collision_sensor_angle)
        # rospy.loginfo('collision at angle WC')
        # rospy.loginfo(collision_angle_wc)
        # rospy.loginfo('avoidance goal WC')
        # rospy.loginfo(avoid_goal)
        return avoid_goal


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
    ranges = [back, left, front, right, up]


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
    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.1, relative=True)

    # # Initate some movement for starting the algorithm
    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=radians(45), duration=3, relative=True)  # move a bit forward before
    time.sleep(3)
    cf_handler.goTo(goal=[0.3, 0.0, 0.0], yaw=0, duration=3, relative=True)  # move a bit forward before
    time.sleep(3)
    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=radians(45), duration=3, relative=True)  # move a bit forward before
    # time.sleep(3)
    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=radians(-45), duration=3, relative=True)  # move a bit forward before
    # time.sleep(3)
    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=radians(0), duration=3, relative=True)  # move a bit forward before
    # time.sleep(3)
    # # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=radians(90), duration=3, relative=True)  # move a bit forward before
    # # time.sleep(3)

    x, y, yaw = 0, 0, 0
    z = initialZ

    global keyboard_flag
    global cj_injection_message, cj_injection_flag
    global front, back, up, left, right, zrange

    dist_threshold = 0.15
    def_duration = 2.0

    last_collision = rospy.Time.now()

    # There's 2 levels of collision protections:
    #   Low level:
    #       The threshold for protecting drone before collision with walls
    # High level:
    #       avoid_collision(), a function that will be called every time theres a
    #       Cj_injection and before that Cj order will go to drone,
    #       we will check if theres a futoristic collision within that path.

    dist_threshold = 0.15  # minimum distance to trigger collission avoidance [meters]

    def_duration = 1.8
    land_duration = 1.5
    avoid_c_duration = 1.0

    try:

        while not rospy.is_shutdown():

            if min(ranges) > 0 and (rospy.Time.now() - last_collision).to_sec() > 2.0:

                # if no range sensors are present all range values will be zero - skip collision
                # minimum time delta between stop msgs due to collision in seconds
                # #ranges order is [back, left, front, right, up]

                if ranges[2] < dist_threshold:
                    rospy.loginfo("front collision avoidance")
                    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.8, relative=True)
                    last_collision = rospy.Time.now()
                    [x, y] = collision_direction_wc(0)
                    cf_handler.goTo(goal=[x, y, 0.0], yaw=0, duration=avoid_c_duration, relative=True)

                elif ranges[0] < dist_threshold:
                    rospy.loginfo("back collision avoidance")
                    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.8, relative=True)
                    last_collision = rospy.Time.now()
                    [x, y] = collision_direction_wc(-1 * pi)
                    cf_handler.goTo(goal=[x, y, 0.0], yaw=0, duration=avoid_c_duration, relative=True)


                elif ranges[3] < dist_threshold:
                    rospy.loginfo("right collision avoidance")
                    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.8, relative=True)
                    last_collision = rospy.Time.now()
                    [x, y] = collision_direction_wc(0.5 * pi)
                    cf_handler.goTo(goal=[x, y, 0.0], yaw=0, duration=avoid_c_duration, relative=True)


                elif ranges[1] < dist_threshold:
                    rospy.loginfo("left collision avoidance")
                    # cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=0.8, relative=True)
                    last_collision = rospy.Time.now()
                    [x, y] = collision_direction_wc(-0.5 * pi)
                    cf_handler.goTo(goal=[x, y, 0.0], yaw=0, duration=avoid_c_duration, relative=True)

                elif ranges[4] < dist_threshold:
                    rospy.loginfo("top collision - landing ")
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    cf_handler.stop()
                    break

            if keyboard_flag is True:
                keyboard_flag = False
                kb_step = 0.3  # meters each cmd_vel message
                cont_rot_yaw = 0
                if kb_x > 0:
                    cf_handler.goTo(goal=[kb_step, 0.0, 0.0], yaw=cont_rot_yaw, duration=def_duration, relative=True)
                elif kb_x < 0:
                    cf_handler.goTo(goal=[-1 * kb_step, 0.0, 0.0], yaw=cont_rot_yaw, duration=def_duration,
                                    relative=True)
                elif kb_yaw < 0:
                    cf_handler.goTo(goal=[0.0, -1 * kb_step, 0.0], yaw=cont_rot_yaw, duration=def_duration,
                                    relative=True)
                elif kb_yaw > 0:
                    cf_handler.goTo(goal=[0.0, kb_step, 0.0], yaw=cont_rot_yaw, duration=def_duration, relative=True)
                elif kb_y > 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=pi / 2, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation
                elif kb_y < 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=-1 * pi / 2, duration=def_duration + 1.0,
                                    relative=True)  # slow down yaw rotation
                elif kb_x == 0 and kb_y == 0:
                    cf_handler.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=1.0, relative=True)  # stop in place

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

                # rospy.logdebug("Cj direction is {}".format(direction))
                rospy.logdebug("Cj duration is {}".format(duration))

                # obstacle_free=avoid_collision()
                # if obstacle_free == True:
                if z <= 0.1:
                    cf_handler.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration)
                    break
                else:
                    rospy.logdebug("going to: {}".format([x, y, z, yaw]))
                    cf_handler.goTo(goal=[x * 1.075, y * 1.075, z], yaw=yaw, duration=duration, relative=False)
                # else:
                #     rospy.logwarn("cannot move - obstacle in the way")

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf_handler.stop()
        # break

    except Exception as e:
        # todo change the while into inner function.
        # cf_handler.stop()
        rospy.loginfo('******* keyboard input exception *******')
        rospy.loginfo(e)

    finally:
        rospy.loginfo('******* EXITING *******')
        cf_handler.stop()


if __name__ == '__main__':
    rospy.init_node('motion', log_level=rospy.DEBUG)  # log_level=rospy.DEBUG

    # last minute addition, listen to keyboard per drone!!
    listen_to_keyboard = rospy.get_param("~listen_to_keyboard")
    rospy.logdebug("listen_to_keyboard: {}".format(listen_to_keyboard))

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
