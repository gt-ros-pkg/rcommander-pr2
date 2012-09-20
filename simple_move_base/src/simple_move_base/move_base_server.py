#! /usr/bin/python
import roslib; roslib.load_manifest('simple_move_base')
import rospy
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import simple_move_base.msg as hm
import tf

import geometry_msgs.msg as gm
import sys
import math
import numpy as np
import actionlib

class MoveBase:

    def __init__(self):
        rospy.init_node('odom_move_base')
        self.tw_pub = rospy.Publisher('base_controller/command', gm.Twist)
        #rospy.Subscriber('simple_move_base', gm.Pose2D, self.pose2d_cb)
        self.tl = tf.TransformListener()

        self.go_xy_server = actionlib.SimpleActionServer('go_xy', hm.GoXYAction, self._go_xy_cb, auto_start=False)
        self.go_xy_server.start()

        self.go_ang_server = actionlib.SimpleActionServer('go_angle', hm.GoAngleAction, self._go_ang_cb, auto_start=False)
        self.go_ang_server.start()

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


    def go_xy(self, target_base, tolerance=.01, max_speed=.1, func=None):
        k = 2.
        self.tl.waitForTransform('base_footprint', 'odom_combined', rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)

        od_T_bf = tfu.transform('odom_combined', 'base_footprint', self.tl)
        target_odom = (od_T_bf * np.row_stack([target_base, np.matrix([0,1.]).T]))[0:2,0]
        #print 'target base', target_base.T
        #print 'target odom', target_odom.T

        robot_odom = np.matrix(tfu.transform('odom_combined', 'base_footprint', self.tl)[0:2,3])
        diff_odom  = target_odom - robot_odom
        mag        = np.linalg.norm(diff_odom)
        rospy.loginfo('go_xy: error %s' % str(mag))
        while not rospy.is_shutdown():
            robot_odom = np.matrix(tfu.transform('odom_combined', 'base_footprint', self.tl)[0:2,3])
            #rospy.loginfo('go_xy: odom %s' % str(robot_odom.T))

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
            #vel_odom = direc * mag * k
            #print mag*k, max_speed, speed

            bf_T_odom = tfu.transform('base_footprint', 'odom_combined', self.tl)
            vel_base = bf_T_odom * np.row_stack([vel_odom, np.matrix([0,0.]).T])
            #pdb.set_trace()
            #rospy.loginfo('vel_base %f %f' % (vel_base[0,0], vel_base[1,0]))

            tw = gm.Twist()
            tw.linear.x = vel_base[0,0]
            tw.linear.y = vel_base[1,0]
            #rospy.loginfo('commanded %s' % str(tw))
            self.tw_pub.publish(tw)
            rate.sleep()

        rospy.loginfo('go_xy: finished go_xy %f' % np.linalg.norm(diff_odom))
        return diff_odom

    ##
    # delta angle
    def go_angle(self, target_base, tolerance=math.radians(5.), max_ang_vel=math.radians(20.), func=None):
        self.tl.waitForTransform('odom_combined', 'base_footprint', rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)
        k = math.radians(80)

        od_T_bf = tfu.transform('odom_combined', 'base_footprint', self.tl)
        target_odom_mat = od_T_bf * tfu.tf_as_matrix( ([0, 0, 0.], tr.quaternion_from_euler(0, 0, target_base)) )
        target_odom = tr.euler_from_quaternion(tfu.matrix_as_tf(target_odom_mat)[1])[2]
        #target_odom = (od_T_bf * np.row_stack([target_base, np.matrix([0,1.]).T]))[0:2,0]

        #current_ang_odom = tr.euler_from_matrix(tfu.transform('base_footprint', 'odom_combined', self.tl)[0:3, 0:3], 'sxyz')[2]
        #target_odom = current_ang_odom + delta_ang

        robot_odom = tfu.transform('odom_combined', 'base_footprint', self.tl)
        current_ang_odom = tr.euler_from_matrix(robot_odom[0:3, 0:3], 'sxyz')[2]
        diff = ut.standard_rad(current_ang_odom - target_odom)
        rospy.loginfo('go_angle: target %.2f' %  np.degrees(target_odom))
        rospy.loginfo('go_angle: current %.2f' %  np.degrees(current_ang_odom))
        rospy.loginfo('go_angle: diff %.2f' %  np.degrees(diff))

        while not rospy.is_shutdown():
            robot_odom = tfu.transform('odom_combined', 'base_footprint', self.tl)
            current_ang_odom = tr.euler_from_matrix(robot_odom[0:3, 0:3], 'sxyz')[2]
            diff = ut.standard_rad(target_odom - current_ang_odom)
            rospy.loginfo('go_angle: current %.2f diff %.2f' %  (np.degrees(current_ang_odom), np.degrees(diff)))
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
            #rospy.loginfo('diff %.3f vel %.3f' % (math.degrees(diff), math.degrees(tw.angular.z)))
            self.tw_pub.publish(tw)
            #rospy.loginfo('commanded %s' % str(tw))
            rate.sleep()
        rospy.loginfo('go_ang: finished %.3f' % math.degrees(diff))
        return diff


if __name__ == '__main__':
    m = MoveBase()
    rospy.loginfo('simple move base server up!')
    rospy.spin()

