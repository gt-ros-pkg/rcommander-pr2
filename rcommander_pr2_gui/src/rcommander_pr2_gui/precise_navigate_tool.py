#import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import pypr2.tf_utils as tfu
import tf.transformations as tr
import pypr2.msg as smb
import math
import actionlib
import smach
from tf_broadcast_server.srv import GetTransforms
import pypr2.pr2_utils as p2u
import numpy as np
import re
import tf

## Convert quanities from se2 to se3
def se2_from_se3(mat):
    t, r = tfu.matrix_as_tf(mat)
    x = t[0]
    y = t[1]
    theta = tr.euler_from_quaternion(r)[2]
    return x,y,theta

ROBOT_FRAME_NAME = '/base_link'

## Moves to a given position using the base but does not have
# awareness of obstacles.  Often is more precise in positioning
# the robot.
# TODO: make this safer by adding in constraints.
class PreciseNavigateTool(tu.ToolBase, p2u.SE3Tool):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate_refined', 
                'Navigate (precise)', PreciseNavigateState)
        p2u.SE3Tool.__init__(self, rcommander.tf_listener)
        self.tf_listener = rcommander.tf_listener

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        group_boxes = self.make_se3_boxes(pbox)
        self.time_out = tu.double_spin_box(pbox, 0., 60., 1.) 

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Update')
        frame_box = self.make_task_frame_box(pbox, self.rcommander)

        formlayout.addRow("&Time Out", self.time_out)
        formlayout.addRow("&Frame", frame_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])

        formlayout.addRow(self.pose_button)
        self.rcommander.connect(self.pose_button, 
                SIGNAL('clicked()'), self.get_current_pose)
        self.reset()

    ## Gets the current base pose from TF and sets the GUI fields
    def get_current_pose(self):
        try:
            frame = str(self.frame_box.currentText())
            self.tf_listener.waitForTransform(frame, ROBOT_FRAME_NAME,
                    rospy.Time(), rospy.Duration(2.))
            p_base = tfu.transform(frame, ROBOT_FRAME_NAME, self.tf_listener)\
                       * tfu.tf_as_matrix(([0., 0., 0., 1.], 
                                tr.quaternion_from_euler(0,0,0)))
            p_base_tf = tfu.matrix_as_tf(p_base)

            for value, vr in zip(p_base_tf[0], \
                    [self.xline, self.yline, self.zline]):
                vr.setValue(value)
            for value, vr in zip(tr.euler_from_quaternion(p_base_tf[1]),\
                    [self.phi_line, self.theta_line, self.psi_line]):
                vr.setValue(np.degrees(value))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                QMessageBox.information(self.rcommander, self.button_name, 
                    'Error looking up frame named '+
                    '"%s".  If this is a task frame, is it highlighted red?'\
                            % str(self.frame_box.currentText()))

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        pose_stamped = self.get_posestamped()
        #rospy.loginfo('new_node ' + str(pose_stamped))
        state = PreciseNavigateState(nname, pose_stamped, 
                self.time_out.value())
        return state

    ## Inherited
    def set_node_properties(self, node):
        #rospy.loginfo('setting properties ' + str(node.pose_stamped))
        self.set_posestamped(node.pose_stamped)
        self.time_out.setValue(node.time_out)

    ## Inherited
    def reset(self):
        for vr in [self.xline, self.yline, self.zline, 
                self.phi_line, self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.frame_box.setCurrentIndex(self.frame_box.findText('/map'))
        self.time_out.setValue(30.)


class PreciseNavigateState(tu.StateBase): 

    ## Constructor
    # @param name Name of robot (string).
    # @param pose_stamped PoseStamped to navigate to.
    # @param time_out Time in seconds (float).
    def __init__(self, name, pose_stamped, time_out):
        tu.StateBase.__init__(self, name)
        self.pose_stamped = pose_stamped
        self.time_out = time_out

    ## Inherited
    def get_smach_state(self):
        return PreciseNavigateSmach(self.pose_stamped, self.time_out)


class PreciseNavigateSmach(smach.State): 

    ## Constructor
    def __init__(self, pose_stamped, time_out):
        smach.State.__init__(self, outcomes = ['succeeded', 
            'preempted', 'failed', 'timed_out'], 
            input_keys = [], output_keys = [])
        self.go_angle_client = actionlib.SimpleActionClient('go_angle', 
                smb.GoAngleAction)
        self.go_xy_client = actionlib.SimpleActionClient('go_xy', 
                smb.GoXYAction)

        self.pose_stamped = pose_stamped
        self.time_out = time_out
        self.CONTROL_FRAME = '/base_footprint'

    ## Inherited
    def set_robot(self, pr2):
        if pr2 != None:
            self.tf_listener = pr2.tf_listener

    ## Inherited
    def execute(self, userdata):
        try:
            #Create goal and send it up here
            self.tf_listener.waitForTransform(self.CONTROL_FRAME,\
                    self.pose_stamped.header.frame_id,  rospy.Time(0),\
                    rospy.Duration(10.))
            bl_T_frame = tfu.tf_as_matrix(\
                    self.tf_listener.lookupTransform(\
                        self.CONTROL_FRAME,\
                        self.pose_stamped.header.frame_id, rospy.Time(0)))

            trans = np.array([self.pose_stamped.pose.position.x, 
                self.pose_stamped.pose.position.y, 
                self.pose_stamped.pose.position.z])
            quat = [self.pose_stamped.pose.orientation.x, 
                    self.pose_stamped.pose.orientation.y, 
                    self.pose_stamped.pose.orientation.z, 
                    self.pose_stamped.pose.orientation.w]

            h_frame = tfu.tf_as_matrix((trans, quat))
            x, y, t = se2_from_se3(bl_T_frame * h_frame)

            xy_goal = smb.GoXYGoal(x,y)
            self.go_xy_client.send_goal(xy_goal)
            result_xy = tu.monitor_goals(self, [self.go_xy_client], 
                    'PreciseNavigateSmach', self.time_out)
            if result_xy != 'succeeded':
                return result_xy

            ang_goal = smb.GoAngleGoal(t)
            self.go_angle_client.send_goal(ang_goal)
            result_ang = tu.monitor_goals(self, [self.go_angle_client], 
                    'PreciseNavigateSmach', self.time_out)

            return result_ang
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            if '/task_frame' == self.pose_stamped.header.frame_id:
                raise tu.TaskFrameError(str(self.__class__), 
                        self.CONTROL_FRAME)
            else:
                raise tu.FrameError(str(self.__class__), 
                        self.pose_stamped.header.frame_id, self.CONTROL_FRAME)


