import rcommander.tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import pypr2.tf_utils as tfu
import tf.transformations as tr
import numpy as np
from object_manipulator.convert_functions import *
import ptp_arm_action.msg as ptp
import math
import geometry_msgs.msg as geo
import actionlib 
import smach
import actionlib_msgs.msg as am
import pypr2.pr2_utils as p2u
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms

## Creates a Cartesian movement goal that tries to guarantee
# positional accuracy.
class PositionPriorityMoveTool(tu.ToolBase, p2u.SE3Tool):

    LEFT_TIP = '/l_gripper_tool_frame'
    RIGHT_TIP = '/r_gripper_tool_frame'

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'position_priority', 
                'Position Priority', PositionPriorityState)
        p2u.SE3Tool.__init__(self, rcommander.tf_listener)
        self.default_frame = '/torso_lift_link'
        self.tf_listener = rcommander.tf_listener

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        frame_box = self.make_task_frame_box(pbox, self.rcommander)
        self.trans_vel_line = tu.double_spin_box(pbox, 0, 1., .02)
        self.rot_vel_line   = tu.double_spin_box(pbox, 0, np.pi, .02)
        group_boxes = self.make_se3_boxes(pbox)

        self.arm_radio_boxes, self.arm_radio_buttons = \
                tu.make_radio_box(pbox, ['Left', 'Right'], 'arm')
        self.timeout_box = QDoubleSpinBox(pbox)
        self.timeout_box.setMinimum(0)
        self.timeout_box.setMaximum(1000.)
        self.timeout_box.setSingleStep(.2)

        self.tolerance_box = QDoubleSpinBox(pbox)
        self.tolerance_box.setMinimum(0)
        self.tolerance_box.setMaximum(1.)
        self.tolerance_box.setSingleStep(.01)

        motion_box = QGroupBox('Motion', pbox)
        motion_layout = QFormLayout(motion_box)
        motion_box.setLayout(motion_layout)
        motion_layout.addRow('Speed', self.trans_vel_line)
        motion_layout.addRow('Speed (rotational)', self.rot_vel_line)
        motion_layout.addRow('&Tolerance', self.tolerance_box)
        motion_layout.addRow("&Arm", self.arm_radio_boxes)
        motion_layout.addRow('&Time Out', self.timeout_box)

        self.pose_buttons_holder = QWidget(pbox)
        self.pose_button = QPushButton(pbox)
        self.lbb_hlayout = QHBoxLayout(self.pose_buttons_holder)
        self.grid_box = QWidget(pbox)
        self.gridlayout = QGridLayout(self.grid_box)
        
        spacer = QSpacerItem(130, 0, QSizePolicy.Minimum, 
                QSizePolicy.Expanding)
        self.pose_button.setText('Update')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), 
                self.get_current_pose)

        formlayout.addRow('Frame', self.frame_box) 
        formlayout.addRow(motion_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])
    
        self.gridlayout.addItem(spacer,0,0)
        self.gridlayout.addWidget(self.pose_button, 0, 1)
        formlayout.addRow(self.grid_box)
        formlayout.addRow(self.pose_buttons_holder)
        self.reset()

    ## Gets the current pose of the gripper tip and sets the GUI
    # fields to it.
    def get_current_pose(self):
        try:
            frame_described_in = str(self.frame_box.currentText())
            arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
            if arm == 'right':
                arm_tip_frame = PositionPriorityMoveTool.RIGHT_TIP
            else:
                arm_tip_frame = PositionPriorityMoveTool.LEFT_TIP
            self.tf_listener.waitForTransform(frame_described_in, 
                    arm_tip_frame, rospy.Time(), rospy.Duration(2.))
            p_arm = self.tf_listener.lookupTransform(frame_described_in, 
                    arm_tip_frame, rospy.Time(0))

            #print 'current pose', p_arm
            for value, vr in zip(p_arm[0], \
                    [self.xline, self.yline, self.zline]):
                vr.setValue(value)
            for value, vr in zip(tr.euler_from_quaternion(p_arm[1]), \
                    [self.phi_line, self.theta_line, self.psi_line]):
                vr.setValue(np.degrees(value))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                QMessageBox.information(self.rcommander, self.button_name, 
                'Error looking up frame named "%s". '+\
                ' If this is a task frame, is it highlighted red?'\
                    % str(self.frame_box.currentText()))

    ## Inherited
    def new_node(self, name=None):
        #make name
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        #other properties
        trans_vel = self.trans_vel_line.value()
        rot_vel   = self.rot_vel_line.value()
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        frame  = str(self.frame_box.currentText())
        time_out = self.timeout_box.value()
        trans_tolerance = self.tolerance_box.value()
        pose_stamped = self.get_posestamped()
        return PositionPriorityState(nname, pose_stamped, trans_vel, 
                rot_vel, arm, time_out, trans_tolerance)

    ## Inherited
    def set_node_properties(self, node):
        self.set_posestamped(node.pose_stamped)
        self.trans_vel_line.setValue(node.trans_vel)
        self.rot_vel_line.setValue(node.rot_vel)

        if node.arm == 'left':
            self.arm_radio_buttons[0].setChecked(True)
        if node.arm == 'right':
            self.arm_radio_buttons[1].setChecked(True)

        self.frame_box.setCurrentIndex(self.frame_box.findText(\
                str(node.pose_stamped.header.frame_id)))
        self.timeout_box.setValue(node.time_out)
        self.tolerance_box.setValue(node.trans_tolerance)

    ## Inherited
    def reset(self):
        self.frame_box.setCurrentIndex(self.frame_box.findText(\
                self.default_frame))
        for vr in [self.xline, self.yline, self.zline, 
                self.phi_line, self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.trans_vel_line.setValue(.02)
        self.rot_vel_line.setValue(.16)
        self.timeout_box.setValue(20)
        self.arm_radio_buttons[1].setChecked(True)
        self.tolerance_box.setValue(.02)

class PositionPriorityState(tu.StateBase): 

    ## Constructor
    # @param name Name of node.
    # @param pose_stamped PoseStamped describing the goal position.
    # @param trans_vel Translational velocity (float).
    # @param rot_vel Rotational velocity (float).
    # @param arm 'left' or 'right'
    # @param time_out Time out in seconds (float).
    # @param trans_tolerance Translational tolerance for failures (float).
    def __init__(self, name, pose_stamped, trans_vel, rot_vel, arm, 
            time_out, trans_tolerance):
        tu.StateBase.__init__(self, name)
        self.pose_stamped  = pose_stamped
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel

        self.arm = arm
        self.time_out = time_out
        self.trans_tolerance = trans_tolerance

    ## Inherited
    def get_smach_state(self):
       return PositionPrioritySmach(self.pose_stamped, self.trans_vel, 
               self.rot_vel, self.arm, self.time_out, 
               self.trans_tolerance)

class PositionPrioritySmach(smach.State):

    ## Constructor
    def __init__(self, pose_stamped, trans_vel, rot_vel, arm, 
                time_out, trans_tolerance):
        smach.State.__init__(self, outcomes = ['succeeded', 
            'preempted', 'timed_out', 'goal_not_reached'], 
            input_keys = [], output_keys = [])

        self.pose_stamped  = pose_stamped
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel
        self.action_client = actionlib.SimpleActionClient(arm + '_ptp', 
                ptp.LinearMovementAction)
        self.time_out = time_out
        self.trans_tolerance = trans_tolerance

    ## Inherited
    def set_robot(self, robot_obj):
        if robot_obj != None:
            self.robot = robot_obj
            self.tf_listener = robot_obj.tf_listener

    ## Given a goal, send it and loop waiting for resutlts.
    def execute_goal(self, goal):
        self.action_client.send_goal(goal)
        state = None

        preempted = False
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('PositionPrioritySmach: preempt requested')
                self.action_client.cancel_all_goals()
                self.service_preempt()
                preempted = True
                break

            state = self.action_client.get_state()
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                break

            r.sleep()

        if preempted:
            return 'preempted'

        if state == am.GoalStatus.SUCCEEDED:
            return 'succeeded'

        elif state == am.GoalStatus.ABORTED:
            result = self.action_client.get_result()
            if result == None or result.message == 'goal_not_reached':
                return 'goal_not_reached'
            elif result.message == 'timed_out':
                return 'timed_out'
            else:
                raise Exception('Unknown failure result: %s' % result.message)
        else:
            raise Exception('Unexpected state reached: %d' % state)


    ## Inherited
    def execute(self, userdata):
        try:
            #Test that this frame is defined
            self.tf_listener.waitForTransform('/base_link', 
                    self.pose_stamped.header.frame_id,
                    rospy.Time(0), rospy.Duration(10.))

            goal = ptp.LinearMovementGoal()
            goal.goal = self.pose_stamped
            goal.trans_vel = self.trans_vel
            goal.rot_vel   = self.rot_vel
            goal.trans_tolerance = self.trans_tolerance
            goal.time_out = self.time_out
            return self.execute_goal(goal)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            if '/task_frame' == self.pose_stamped.header.frame_id:
                raise tu.TaskFrameError(str(self.__class__), '/base_link')
            else:
                raise tu.FrameError(str(self.__class__), 
                        self.pose_stamped.header.frame_id, '/base_link')

