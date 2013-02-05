import rcommander.tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import pypr2.tf_utils as tfu
import tf.transformations as tr
import move_base_msgs.msg as mm
import math
import pypr2.pr2_utils as p2u
import numpy as np
import smach
import actionlib
import tf
#Frame that the robot's base position is in.
ROBOT_FRAME_NAME = '/base_link'

## Sends the PR2 to a given position & orientation using the navstack
class NavigateTool(tu.ToolBase, p2u.SE3Tool):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate', \
                'Navigate (planner)', NavigateState)
        p2u.SE3Tool.__init__(self, rcommander.tf_listener)
        self.tf_listener = rcommander.tf_listener

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        group_boxes = self.make_se3_boxes(pbox)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Update')
        frame_box = self.make_task_frame_box(pbox, self.rcommander)

        formlayout.addRow("&Frame", frame_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])
        formlayout.addRow(self.pose_button)
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), 
                self.get_current_pose_cb)
        pbox.update()
        self.reset()

    ## Callback for get current pose button.  Fetches the robot's
    # current 2D pose
    def get_current_pose_cb(self):
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
            for value, vr in zip(tr.euler_from_quaternion(p_base_tf[1]), \
                    [self.phi_line, self.theta_line, self.psi_line]):
                vr.setValue(np.degrees(value))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                QMessageBox.information(self.rcommander, self.button_name, 
                    'Error looking up frame named '+ \
                    '"%s".  If this is a task frame, is it highlighted red?'\
                    % str(self.frame_box.currentText()))

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        pose_stamped = self.get_posestamped()
        state = NavigateState(nname, pose_stamped)
        return state

    ## Inherited
    def set_node_properties(self, node):
        self.set_posestamped(node.pose_stamped)

    ## Inherited
    def reset(self):
        for vr in [self.xline, self.yline, self.zline, \
                self.phi_line, self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.frame_box.setCurrentIndex(self.frame_box.findText('/map'))


class NavigateState(tu.StateBase): 

    ## Constructor
    # @param name Name of node.
    # @param pose_stamped PoseStamped to navigate to.
    def __init__(self, name, pose_stamped):
        tu.StateBase.__init__(self, name)
        self.pose_stamped = pose_stamped

    ## Inherited
    def get_smach_state(self):
        return NavigateSmach(self.pose_stamped)


class NavigateSmach(smach.State): 

    ## Constructor
    # @param pose_stamped
    def __init__(self, pose_stamped):
        smach.State.__init__(self, outcomes = \
                ['succeeded', 'preempted', 'aborted', 'failed'], 
                input_keys = [], output_keys = [])
        self.pose_stamped = pose_stamped
        self.move_base_client = actionlib.SimpleActionClient(\
                'move_base', mm.MoveBaseAction)

    ## Inherited
    def set_robot(self, pr2):
        if pr2 != None:
            self.tf_listener = pr2.tf_listener

    ## Inherited
    def execute(self, userdata):
        rospy.loginfo('waiting for transform')

        try:
            self.tf_listener.waitForTransform(\
                    self.pose_stamped.header.frame_id, 
                    '/map', rospy.Time(0), rospy.Duration(10.))
            self.pose_stamped.header.stamp = rospy.Time()
            ps = self.tf_listener.transformPose('/map', self.pose_stamped)
            g = mm.MoveBaseGoal()
            p = g.target_pose
            
            p.header.frame_id = ps.header.frame_id
            p.header.stamp = rospy.get_rostime()

            p.pose.position.x = ps.pose.position.x
            p.pose.position.y = ps.pose.position.y
            p.pose.position.z = 0 
       
            raw_angles = tr.euler_from_quaternion(\
                    [ps.pose.orientation.x, ps.pose.orientation.y,
                     ps.pose.orientation.z, ps.pose.orientation.w])
            quat = tr.quaternion_from_euler(0., 0., raw_angles[2])
            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]

            self.move_base_client.send_goal(g)
            result = tu.monitor_goals(self, [self.move_base_client], 
                    'NavigateSmach', 60*60)
            if result == 'failed':
                return 'aborted'
            return result
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            if '/task_frame' == self.pose_stamped.header.frame_id:
                raise tu.TaskFrameError(str(self.__class__), '/map')
            else:
                raise tu.FrameError(str(self.__class__), 
                        self.pose_stamped.header.frame_id, '/map')
        
