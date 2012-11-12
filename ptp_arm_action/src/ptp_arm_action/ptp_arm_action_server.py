#! /usr/bin/python
import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib

import geometry_msgs.msg as gm
import ptp_arm_action.msg as ptp

#from object_manipulator.convert_functions import *
from math import sqrt, pi, fabs
import tf_utils as tfu
import numpy as np
#import scipy
import math
import tf.transformations as tr
import tf
from pycontroller_manager.pycontroller_manager import ControllerManager
import object_manipulator.convert_functions as cf

##stamp a message by giving it a header with a timestamp of now
def stamp_msg(msg, frame_id):
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()

##convert a 4x4 scipy matrix to a Pose message 
#(first premultiply by transform if given)
def mat_to_pose(mat, transform = None):
    if transform != None:
        mat = transform * mat
    pose = gm.Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

##make a PoseStamped out of a Pose
def stamp_pose(pose, frame_id):
    pose_stamped = gm.PoseStamped()
    stamp_msg(pose_stamped, frame_id)
    pose_stamped.pose = pose
    return pose_stamped


##change the frame of a PoseStamped
def change_pose_stamped_frame(tf_listener, pose, frame):

    #convert the PoseStamped to the desired frame, if necessary
    if pose.header.frame_id != frame:
        pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(frame, pose.header.frame_id, 
                pose.header.stamp, rospy.Duration(5))
        try:
            trans_pose = tf_listener.transformPose(frame, pose)
        except rospy.ServiceException, e:
            print "pose:\n", pose
            print "frame:", frame
            rospy.logerr("change_pose_stamped_frame: error in transforming "\
                        +"pose from " + pose.header.frame_id + " to "\
                        + frame + "error msg: %s"%e)
            return None
    else:
        trans_pose = pose

    return trans_pose


def pose_to_mat(pose):
    quat = [pose.orientation.x, pose.orientation.y, 
            pose.orientation.z, pose.orientation.w]
    pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat

##
# Measures the distance between two pose stamps, independently factors out
# distance in rotation and distance in translation. Converts both poses into
# the same coordinate frame before comparison.
def pose_distance(ps_a, ps_b, tflistener):
    #put both into the same frame
    ps_a_fb = change_pose_stamped_frame(tflistener, ps_a, ps_b.header.frame_id)
    g_T_a = pose_to_mat(ps_a_fb.pose)
    g_T_b = pose_to_mat(ps_b.pose)

    a_T_b = g_T_a**-1 * g_T_b

    desired_trans = a_T_b[0:3, 3].copy()
    a_T_b[0:3, 3] = 0
    desired_angle, desired_axis, _ = \
        tf.transformations.rotation_from_matrix( a_T_b)
    return desired_trans, desired_angle, desired_axis

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) \
            * np.matrix(tr.quaternion_matrix(tup[1])) 


class PTPArmActionServer:

    def __init__(self, name, arm):
        if arm == 'left':
            self.controller = 'l_cart'
            #self.joint_controller = 'l_arm_controller'
            self.controller_frame = rospy.get_param('/l_cart/tip_name')
            #self.tool_frame = rospy.get_param('/l_cart/tip_name')
            self.tool_frame = 'l_gripper_tool_frame'
        elif arm == 'right':
            self.controller = 'r_cart'
            #self.joint_controller = 'r_arm_controller'
            self.controller_frame = rospy.get_param('/r_cart/tip_name')
            #self.tool_frame = rospy.get_param('/r_cart/tip_name')
            self.tool_frame = 'r_gripper_tool_frame'
        else:
            raise RuntimeError('Invalid parameter for arm: %s' % arm)

        self.arm = arm
        self.target_pub = rospy.Publisher(self.controller \
                + '/command_pose', gm.PoseStamped)

        self.tf = tf.TransformListener()
        self.rot_tolerance = math.radians(
                rospy.get_param("~rotation_tolerance"))
        self.stall_time = rospy.get_param("~stall_time")

        self.controller_manager = ControllerManager()
        self._action_name = name
        self.linear_movement_as = actionlib.SimpleActionServer(\
                self._action_name, ptp.LinearMovementAction, 
				execute_cb=self.action_cb, auto_start=False)
        self.linear_movement_as.start()

        rospy.loginfo('Action name: %s Arm: %s'% (self._action_name, self.arm))


    def action_cb(self, msg):
        rospy.loginfo('message that we got:\n' + str(msg))
        self.controller_manager.cart_mode(self.arm)
        trans_tolerance = msg.trans_tolerance 
        time_out = msg.time_out 
        self.rot_tolerance = math.radians(
                rospy.get_param("~rotation_tolerance"))

        success = False
        r = rospy.Rate(100)

        goal_ps = msg.goal
        trans_vel = msg.trans_vel
        rot_vel = msg.rot_vel
        if trans_vel <= 0:
            trans_vel = .02
        if rot_vel <= 0:
            rot_vel = pi/20.

        tstart = rospy.get_time()
        tmax = tstart + time_out
        rospy.loginfo('Goal is x %f y %f z %f in %s'\
                % (goal_ps.pose.position.x, goal_ps.pose.position.y, 
            goal_ps.pose.position.z, goal_ps.header.frame_id))

        goal_torso = change_pose_stamped_frame(self.tf, goal_ps, 
                'torso_lift_link')
        rospy.loginfo('BEFORE TIP Goal is x %f y %f z %f in %s' \
                % (goal_torso.pose.position.x, goal_torso.pose.position.y, 
            goal_torso.pose.position.z, goal_torso.header.frame_id))

        rospy.loginfo('Tool Frame %s Controller Frame %s' \
                % (self.tool_frame, self.controller_frame))

        tll_T_tip = pose_to_mat(goal_torso.pose)
        tip_T_w = tf_as_matrix(self.tf.lookupTransform(self.tool_frame, 
            self.controller_frame,  rospy.Time(0)))
        tll_T_w = tll_T_tip * tip_T_w
        goal_torso = stamp_pose(mat_to_pose(tll_T_w), 'torso_lift_link')

        rospy.loginfo('AFTER TIP Goal is x %f y %f z %f in %s' \
                % (goal_torso.pose.position.x, goal_torso.pose.position.y, 
            goal_torso.pose.position.z, goal_torso.header.frame_id))

        verbose = False
        time_ang = None
        min_ang_error = None
        time_trans = None
        min_trans_error = None
        timed_out = False

        while not rospy.is_shutdown():
            cur_time = rospy.get_time()

            gripper_matrix = tfu.tf_as_matrix(
                    self.tf.lookupTransform('torso_lift_link', 
                        self.controller_frame, rospy.Time(0)))
            gripper_ps = stamp_pose(mat_to_pose(gripper_matrix), 
                                    'torso_lift_link')
            #Someone preempted us!
            if self.linear_movement_as.is_preempt_requested():
                #Stop our motion
                self.target_pub.publish(stamp_pose(gripper_ps.pose, 
                    gripper_ps.header.frame_id))
                self.linear_movement_as.set_preempted()
                rospy.loginfo('action_cb: PREEMPTED!')
                break

            #Calc feedback
            if verbose:
                print 'cur_pose %.3f %.3f %.3f, rot %.3f %.3f %.3f %.3f in %s'\
                        % (gripper_ps.pose.position.x, 
                           gripper_ps.pose.position.y, 
                           gripper_ps.pose.position.z, 
                           gripper_ps.pose.orientation.x, 
                           gripper_ps.pose.orientation.y, 
                           gripper_ps.pose.orientation.z, 
                           gripper_ps.pose.orientation.w, 
                           gripper_ps.header.frame_id)
                print 'gl_pose %.3f %.3f %.3f, rot %.3f %.3f %.3f %.3f in %s' \
                        % (goal_torso.pose.position.x, 
                           goal_torso.pose.position.y, 
                           goal_torso.pose.position.z, 
                           goal_torso.pose.orientation.x, 
                           goal_torso.pose.orientation.y, 
                           goal_torso.pose.orientation.z, 
                           goal_torso.pose.orientation.w, 
                           goal_torso.header.frame_id)

            trans, ang, _ = pose_distance(gripper_ps, goal_torso, self.tf)
            feedback = ptp.LinearMovementFeedback(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
            self.linear_movement_as.publish_feedback(feedback)

            #Reached goal
            trans_mag = np.linalg.norm(trans)
            if verbose:
                print trans.T, 'trans_mag', trans_mag, 'ang', abs(ang), 'rot toler', self.rot_tolerance

            if min_trans_error == None or min_trans_error == None:
                min_trans_error = trans_mag
                min_ang_error = abs(ang)
                time_ang = cur_time
                time_trans = cur_time

            if trans_mag < min_trans_error:
                min_trans_error = trans_mag
                time_trans = cur_time

            if abs(ang) < min_ang_error:
                min_ang_error = abs(ang)
                time_ang = cur_time

            if trans_tolerance > trans_mag and self.rot_tolerance > abs(ang):
                rospy.loginfo('action_cb: reached goal.')
                break

            #Timed out! is this a failure?
            if cur_time > tmax:
                rospy.loginfo('action_cb: timed out.')
                timed_out = True
                break

            #if it has been a while since we made progress
            if trans_mag > min_trans_error and (cur_time - time_trans) > self.stall_time:
                rospy.loginfo('action_cb: stalled.')
                break

            # tends to not stall on angle so don't look out for this case
            #if abs(ang) > min_ang_error and (cur_time - time_ang) > self.stall_time:
            #    rospy.loginfo('action_cb: stalled.')
            #    break

            #Send controls
            clamped_target = self.clamp_pose(goal_torso, trans_vel, rot_vel, ref_pose=gripper_ps)
            if verbose:
                print 'clamped_target', clamped_target.pose.position.x, clamped_target.pose.position.y, 
                print clamped_target.pose.position.z, clamped_target.header.frame_id, '\n'

            #break
            self.target_pub.publish(clamped_target)
            #break

        trans, ang, _ = pose_distance(gripper_ps, goal_torso, self.tf)
        result = ptp.LinearMovementResult(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]), 'unknown')
        if trans_tolerance > np.linalg.norm(trans):
            rospy.loginfo( 'SUCCEEDED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            result.message = 'succeeded'
            self.linear_movement_as.set_succeeded(result)
        else:
            rospy.loginfo('ABORTED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            if timed_out:
                result.message = 'timed_out'
            else:
                result.message = 'goal_not_reached'
            self.linear_movement_as.set_aborted(result)

    
    def clamp_pose(self, desired_pose, max_trans, max_rot, ref_pose):
        current_pose_d = change_pose_stamped_frame(self.tf, ref_pose, desired_pose.header.frame_id) 
        g_T_c  = pose_to_mat(current_pose_d.pose)
            
        desired_trans, desired_angle, desired_axis = pose_distance(ref_pose, desired_pose, self.tf)
        desired_trans_mag = np.linalg.norm(desired_trans)
        frac_trans = fabs(desired_trans_mag / max_trans)
        frac_rot = fabs(desired_angle / max_rot)

        if frac_trans <= 1 and frac_rot <= 1:
            return desired_pose
        frac = max(frac_rot, frac_trans)

        clamped_angle = desired_angle / frac
        clamped_trans = desired_trans / frac


        c_T_d = np.matrix(tf.transformations.rotation_matrix(clamped_angle, desired_axis))
        c_T_d[0:3, 3] = clamped_trans
        g_T_d = g_T_c * c_T_d
        clamped_pose = stamp_pose(mat_to_pose(g_T_d), desired_pose.header.frame_id)
        clamped_pose.header.stamp = rospy.Time.now()
        return clamped_pose

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        arm = 'left'
    else:
        arm = sys.argv[1]

    rospy.init_node('ptp')
    left_as = PTPArmActionServer(arm +'_ptp', arm)
    rospy.loginfo('PTP action server started.')
    rospy.spin()

