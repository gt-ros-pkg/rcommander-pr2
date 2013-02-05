#! /usr/bin/python
import roslib; roslib.load_manifest('pypr2')
import rospy
import tf.transformations as tr
import pypr2.tf_utils as tfu
import pypr2.pr2_utils as ut
import pypr2.msg as hm
import tf

import geometry_msgs.msg as gm
import sys
import math
import numpy as np
import actionlib

## Contains proportional controllers for base translation and rotation
# (does not pay attention to obstacles).
class MoveBase:

    ## Constructor
    def __init__(self):
        rospy.init_node('odom_move_base')
        self.tw_pub = rospy.Publisher('base_controller/command', gm.Twist)
        self.tl = tf.TransformListener()

        self.go_xy_server = actionlib.SimpleActionServer('go_xy', 
                hm.GoXYAction, self._go_xy_cb, auto_start=False)
        self.go_xy_server.start()

        self.go_ang_server = actionlib.SimpleActionServer('go_angle', 
                hm.GoAngleAction, self._go_ang_cb, auto_start=False)
        self.go_ang_server.start()

    ## Callback for GoXYAction
    # @param goal Goal for action.
    def _go_xy_cb(self, goal):
        rospy.loginfo('received go_xy goal %f %f' % (goal.x, goal.y))
        def send_feed_back(error):
            feed_back_msg = hm.GoXYFeedback()
            feed_back_msg.error = np.linalg.norm(error)
            self.go_xy_server.publish_feedback(feed_back_msg)
        error = self.go_xy(np.matrix([goal.x, goal.y]).T, func=send_feed_back)
        result = hm.GoXYResult()
        result.error = np.linalg.norm(error)
        self.go_xy_server.set_succeeded(result)


    ## Callback for GoAngleAction
    # @param goal Goal for action.
    def _go_ang_cb(self, goal):
        rospy.loginfo('received go_angle goal %f' % (goal.angle))

        def send_feed_back(error):
            feed_back_msg = hm.GoAngleFeedback()
            feed_back_msg.error = error
            self.go_ang_server.publish_feedback(feed_back_msg)

        error = self.go_angle(goal.angle, func=send_feed_back)
        result = hm.GoAngleResult()
        result.error = error
        self.go_ang_server.set_succeeded(result)


    ## Sends the robot to a 2D base pose
    # @param target_base 1x2 matrix representing a 2D position
    # @param tolerance how close to get to goal
    # @param max_speed Maximum speed to allow.
    # @param func stopping function that takes in a float (angular difference)
    def go_xy(self, target_base, tolerance=.01, max_speed=.1, func=None):
        k = 2.
        self.tl.waitForTransform('base_footprint', 'odom_combined', 
                rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)

        od_T_bf = tfu.transform('odom_combined', 'base_footprint', self.tl)
        target_odom = (od_T_bf * np.row_stack([target_base, 
                                            np.matrix([0,1.]).T]))[0:2,0]

        robot_odom = np.matrix(tfu.transform('odom_combined', 
                            'base_footprint', self.tl)[0:2,3])
        diff_odom  = target_odom - robot_odom
        mag        = np.linalg.norm(diff_odom)
        rospy.loginfo('go_xy: error %s' % str(mag))
        while not rospy.is_shutdown():
            robot_odom = np.matrix(tfu.transform('odom_combined', 
                'base_footprint', self.tl)[0:2,3])

            diff_odom  = target_odom - robot_odom
            mag        = np.linalg.norm(diff_odom)
            if func != None:
                func(diff_odom)
            if mag < tolerance:
                break

            if self.go_xy_server.is_preempt_requested():
                self.go_xy_server.set_preempted()
                break

            direc    = diff_odom / mag
            speed = min(mag * k, max_speed)
            vel_odom = direc * speed

            bf_T_odom = tfu.transform('base_footprint', 
                    'odom_combined', self.tl)
            vel_base = bf_T_odom * np.row_stack([vel_odom, np.matrix([0,0.]).T])

            tw = gm.Twist()
            tw.linear.x = vel_base[0,0]
            tw.linear.y = vel_base[1,0]
            self.tw_pub.publish(tw)
            rate.sleep()

        rospy.loginfo('go_xy: finished go_xy %f' % np.linalg.norm(diff_odom))
        return diff_odom

    ## Send the base to an angle in base_footprint rame.
    # @param target_base desired angle in odom_combined frame
    # @param tolerance close should we be before considering that
    #               we've reached the target
    # @param max_ang_vel maximum angular velocity
    # @@param func stopping function that takes in differences 
    def go_angle(self, target_base, tolerance=math.radians(5.), 
            max_ang_vel=math.radians(20.), func=None):
        self.tl.waitForTransform('odom_combined', 
                'base_footprint', rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)
        k = math.radians(80)

        od_T_bf = tfu.transform('odom_combined', 'base_footprint', self.tl)
        target_odom_mat = od_T_bf * tfu.tf_as_matrix(([0, 0, 0.], 
            tr.quaternion_from_euler(0, 0, target_base)))
        target_odom = tr.euler_from_quaternion(tfu.matrix_as_tf(
            target_odom_mat)[1])[2]

        robot_odom = tfu.transform('odom_combined', 'base_footprint', self.tl)
        current_ang_odom = tr.euler_from_matrix(robot_odom[0:3, 0:3], 'sxyz')[2]
        diff = ut.standard_rad(current_ang_odom - target_odom)
        rospy.loginfo('go_angle: target %.2f' %  np.degrees(target_odom))
        rospy.loginfo('go_angle: current %.2f' %  np.degrees(current_ang_odom))
        rospy.loginfo('go_angle: diff %.2f' %  np.degrees(diff))

        while not rospy.is_shutdown():
            robot_odom = tfu.transform('odom_combined', 'base_footprint', 
                    self.tl)
            current_ang_odom = tr.euler_from_matrix(\
                    robot_odom[0:3, 0:3], 'sxyz')[2]
            diff = ut.standard_rad(target_odom - current_ang_odom)
            rospy.loginfo('go_angle: current %.2f diff %.2f'\
                    % (np.degrees(current_ang_odom), np.degrees(diff)))
            p = k * diff
            if func != None:
                func(diff)
            if np.abs(diff) < tolerance:
                break

            if self.go_ang_server.is_preempt_requested():
                self.go_ang_server.set_preempted()
                break

            tw = gm.Twist()
            vels = [p, np.sign(p) * max_ang_vel]
            tw.angular.z = vels[np.argmin(np.abs(vels))]
            self.tw_pub.publish(tw)
            rate.sleep()
        rospy.loginfo('go_ang: finished %.3f' % math.degrees(diff))
        return diff


if __name__ == '__main__':
    m = MoveBase()
    rospy.loginfo('simple move base server up!')
    rospy.spin()

