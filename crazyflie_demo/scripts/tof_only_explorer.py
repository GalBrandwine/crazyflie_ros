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
from math import pi
from tf.transformations import euler_from_quaternion

import crazyflie
# from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData

speed = 0.25

global cj_injection_flag, cj_injection_message
cj_injection_flag = False
cj_injection_message = None


# def check_direction():
#     global listener, tfBuffer, cj_injection_message
#
#     speed = 0.20  # default speed m/s
#     rot_speed = 0.5  # default rot speed sec/radian
#     min_duration = 2.0  # minimum time [sec] for single trajectory
#     duration = default_duration = 2  # sec
#     trans = None
#     try:
#         trans = tfBuffer.lookup_transform(prefix, 'world', rospy.Time(0))
#     except:
#         rospy.logwarn("tf lookup -- {} not found".format(prefix))
#     if trans != None:
#         cj_local_coord = PoseStamped()
#         cj_local_coord = tf2_geometry_msgs.do_transform_pose(cj_injection_message, trans)
#         # rospy.loginfo(cj_local_coord)
#         heading = atan2(cj_local_coord.pose.position.y, cj_local_coord.pose.position.x)
#         # rospy.loginfo(heading)
#
#         distance = sqrt(pow(cj_local_coord.pose.position.x, 2) + pow(cj_local_coord.pose.position.y, 2))
#         duration = distance / speed  # #calculate required time for this motion
#         # rospy.logdebug("in check_direction: before if: [{},{}]".format(heading, duration))
#         if duration < min_duration:
#             duration = min_duration
#
#         q = (cj_local_coord.pose.orientation.x,
#              cj_local_coord.pose.orientation.y,
#              cj_local_coord.pose.orientation.z,
#              cj_local_coord.pose.orientation.w)
#
#         euler = euler_from_quaternion(q, axes='sxyz')
#
#         rotation_yaw = abs(euler[2])
#
#         duration_yaw = rotation_yaw / rot_speed
#         # rospy.logdebug("in check_direction: duration_yaw: {}".format(duration_yaw))
#         # rospy.logdebug("in check_direction: duration: {}".format(duration))
#         if duration_yaw > duration:
#             duration = duration_yaw
#
#         # rospy.logdebug("in check_direction. returning: [{},{}]".format(heading, duration))
#         return [heading, duration]
#     else:
#         rospy.logerr("in check_direction: transform is None")
#         return [0, duration]
#
#
# def collision_direction_wc(collision_sensor_angle):
#     evade_distance = 0.07  # distance to go opposite direction of wall - meters
#
#     global prefix
#     trans = None
#     try:
#         trans = tfBuffer.lookup_transform(prefix, 'world', rospy.Time(0))
#     except:
#         rospy.logwarn("tf lookup -- {} not found".format(prefix))
#     if trans != None:
#         q = (trans.transform.rotation.x,
#              trans.transform.rotation.y,
#              trans.transform.rotation.z,
#              trans.transform.rotation.w)
#
#         euler = euler_from_quaternion(q, axes='sxyz')
#         yaw = euler[2]
#
#         collision_angle_wc = collision_sensor_angle + yaw
#         if collision_angle_wc < -2 * pi:
#             collision_angle_wc += 2 * pi
#
#         if collision_angle_wc > 2 * pi:
#             collision_angle_wc -= 2 * pi
#
#         y = evade_distance * sin(collision_angle_wc)
#         x = evade_distance * cos(collision_angle_wc)
#         avoid_goal = [-x, y]
#         return avoid_goal


def handler(cf_handler):
    r = rospy.Rate(5)
    time.sleep(0.5)
    cf_handler.cf.takeoff(targetHeight=cf_handler.initialZ, duration=4.0)
    time.sleep(5.0)

    global cj_injection_message, cj_injection_flag

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
        # move a bit forward
        cf_handler.cf_handler.goTo(goal=[0.3, 0.0, 0.0], yaw=0, duration=avoid_c_duration,
                                   relative=True)
        time.sleep(3)

        while not rospy.is_shutdown():

            # todo add while ranges < distance got_to....
            # move forward until right corridor
            while cf_handler.ranges[3] <= 0.5:
                cf_handler.cf_handler.goTo(goal=[0.3, 0.0, 0.0], yaw=0, duration=avoid_c_duration,
                                           relative=True)
                rospy.logdebug("Right range is: {}".format(cf_handler.ranges[3]))

            break

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

            if cf_handler.keyboard_flag is True:
                cf_handler.keyboard_flag = False
                kb_step = 0.3  # meters each cmd_vel message

                [kb_x, kb_y, kb_z, kb_yaw] = cf_handler.keyboard

                cont_rot_yaw = 0
                if kb_x > 0:
                    cf_handler.cf.goTo(goal=[kb_step, 0.0, 0.0], yaw=cont_rot_yaw, duration=def_duration, relative=True)
                elif kb_x < 0:
                    cf_handler.cf.goTo(goal=[-1 * kb_step, 0.0, 0.0], yaw=cont_rot_yaw, duration=def_duration,
                                       relative=True)
                elif kb_yaw < 0:
                    cf_handler.cf.goTo(goal=[0.0, -1 * kb_step, 0.0], yaw=cont_rot_yaw, duration=def_duration,
                                       relative=True)
                elif kb_yaw > 0:
                    cf_handler.cf.goTo(goal=[0.0, kb_step, 0.0], yaw=cont_rot_yaw, duration=def_duration, relative=True)
                elif kb_y > 0:
                    cf_handler.cf.goTo(goal=[0.0, 0.0, 0.0], yaw=pi / 2, duration=def_duration + 1.0,
                                       relative=True)  # slow down yaw rotation
                elif kb_y < 0:
                    cf_handler.cf.goTo(goal=[0.0, 0.0, 0.0], yaw=-1 * pi / 2, duration=def_duration + 1.0,
                                       relative=True)  # slow down yaw rotation
                elif kb_x == 0 and kb_y == 0:
                    cf_handler.cf.goTo(goal=[0.0, 0.0, 0.0], yaw=0, duration=1.0, relative=True)  # stop in place

                if kb_z != 0:
                    cf_handler.cf.land(targetHeight=0.0, duration=land_duration)
                    time.sleep(land_duration - 0.5)
                    cf_handler.cf.stop()

            # If Cj injection received:
            # if cj_injection_flag is True:
            #     cf_handler.cj_injection_flag = False
            #     # get Cj_injection in drone coordinates
            #     [x, y, z, yaw] = get_xyz_yaw(cj_injection_message)
            #     [direction, duration] = check_direction()
            #
            #     # rospy.logdebug("Cj direction is {}".format(direction))
            #     rospy.logdebug("Cj duration is {}".format(duration))
            #
            #     # obstacle_free=avoid_collision()
            #     # if obstacle_free == True:
            #     cf_handler.goTo(goal=[x * 1.075, y * 1.075, z], yaw=yaw, duration=duration, relative=False)
            #     # else:
            #     #     rospy.logwarn("cannot move - obstacle in the way")

            r.sleep()

        rospy.loginfo('********EXITING*********')
        cf_handler.cf.stop()
        # break

    except Exception as e:
        cf_handler.stop()
        rospy.loginfo('******* keyboard input exception *******')
        rospy.loginfo(e)

    finally:
        rospy.loginfo('******* EXITING *******')
        cf_handler.stop()


class Cf:
    """ A class for holding a drone (listeners, publishers data..). """

    def __init__(self, prefix, initialZ=0.35):
        self.cf = crazyflie.Crazyflie("/" + prefix, "world")
        self.initialZ = initialZ
        self.ranges_sub = rospy.Subscriber('/' + prefix + '/log_ranges', GenericLogData, self.get_ranges)
        self.Cj_injection_sub = rospy.Subscriber('/' + prefix + '/Cj_injcetor', PoseStamped, self.Cj_injector)
        self.keyboard_listener = rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)

        self.tfBuffer = tf2_ros.Buffer()  # initialize tf buffer for transform lookupupdate_params
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.cj_injection_flag = False
        self.Cj_injection = [None, None, None, None, None]
        self.ranges = [None, None, None, None, None]
        self.keyboard_flag = False
        self.keyboard = [None, None, None, None, None]

    def set_param(self):
        """Simple cf.setParam wrapper. """

        self.cf.setParam("commander/enHighLevel", 1)
        self.cf.setParam("stabilizer/estimator", 2)  # Use EKF

        self.cf.setParam("ctrlMel/kp_z", 0.8)  # reduce z wobble - default 1.25
        self.reset_kalman()

    def reset_kalman(self):
        # reset kalman
        self.cf.setParam("kalman/initialX", 0)
        self.cf.setParam("kalman/initialY", 0)
        self.cf.setParam("kalman/initialZ", 0)
        self.cf.setParam("kalman/resetEstimation", 1)

        time.sleep(0.2)
        self.cf.setParam("kalman/resetEstimation", 0)
        time.sleep(0.5)
        self.cf.setParam("stabilizer/controller", 2)  # 2=Use mellinger controller
        time.sleep(0.2)

    def get_ranges(self, msg):
        # todo continue with this late!!!!!! move global into object
        global front, back, up, left, right, zrange, ranges
        weight_old = 0.65  # the weight given to old values in filter 0-1
        weight_new = 1.0 - weight_old
        front = weight_old * front + weight_new * msg.values[
            0] / 1000  # low-pass filter on range inputs used for collision
        back = weight_old * back + weight_new * msg.values[1] / 1000
        up = weight_old * up + weight_new * msg.values[2] / 1000
        left = weight_old * left + weight_new * msg.values[3] / 1000
        right = weight_old * right + weight_new * msg.values[4] / 1000
        # zrange = msg.values[5] / 1000
        self.ranges = [back, left, front, right, up]

    def twist_callback(self, msg):
        kb_x = msg.linear.x
        kb_y = msg.linear.y
        kb_z = msg.linear.z
        kb_yaw = msg.angular.z

        self.keyboard_flag = True
        self.keyboard = [kb_x, kb_y, kb_z, kb_yaw]

    def Cj_injector(self, msg):
        """Callback for each Cj_injection. """

        self.cj_injection_flag = True
        """Transform Cj injection into drone coordinates"""
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform(prefix + '_takeoff', 'world', rospy.Time(0))
        except:
            rospy.logwarn("tf lookup -- {} not found".format(prefix + '_takeoff'))
        if trans is not None:
            cj_local_coord = tf2_geometry_msgs.do_transform_pose(msg, trans)

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

            self.Cj_injection = [x, y, z, yaw]
        else:
            rospy.logwarn("transform is None!")
            self.Cj_injection = [None, None, None, None, None]


if __name__ == '__main__':
    rospy.init_node('maze_TOF_explorer', log_level=rospy.DEBUG)  # log_level=rospy.DEBUG

    prefix = rospy.get_param("~tf_prefix")
    # rospy.Subscriber('/' + prefix + '/log_ranges', GenericLogData, get_ranges)
    # rospy.Subscriber('/' + prefix + '/Cj_injcetor', PoseStamped, Cj_injector)

    # rospy.Subscriber("/cmd_vel", Twist, twist_callback)

    cf = Cf(prefix, initialZ=0.35)
    rospy.wait_for_service("/" + prefix + '/update_params')
    rospy.loginfo("found update_params service")

    cf.set_param()
    time.sleep(0.2)

    rospy.loginfo("launching threads")
    handler(cf)

    rospy.spin()
