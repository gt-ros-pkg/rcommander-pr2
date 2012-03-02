#import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rcommander.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import numpy as np
from object_manipulator.convert_functions import *
import ptp_arm_action.msg as ptp
import math
import geometry_msgs.msg as geo
import actionlib 
import smach
import actionlib_msgs.msg as am
import pr2_utils as p2u
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms

class PositionPriorityMoveTool(tu.ToolBase, p2u.SE3Tool):

    LEFT_TIP = '/l_gripper_tool_frame'
    RIGHT_TIP = '/r_gripper_tool_frame'
    #LEFT_TIP = rospy.get_param('/l_cart/tip_name')
    #RIGHT_TIP = rospy.get_param('/r_cart/tip_name')

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'position_priority', 'Position Priority', PositionPriorityState)
        p2u.SE3Tool.__init__(self)
        self.default_frame = '/torso_lift_link'
        self.tf_listener = rcommander.tf_listener
        #self.suggested_frames = ['/base_link', '/torso_lift_link', '/l_gripper_tool_frame', '/r_gripper_tool_frame']
        #self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        frame_box = self.make_task_frame_box(pbox)
        self.trans_vel_line = tu.double_spin_box(pbox, 0, 1., .02)
        self.rot_vel_line   = tu.double_spin_box(pbox, 0, np.pi, .02)#QLineEdit(pbox)
        group_boxes = self.make_se3_boxes(pbox)

        self.arm_radio_boxes, self.arm_radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'arm')
        self.timeout_box = QDoubleSpinBox(pbox)
        self.timeout_box.setMinimum(0)
        self.timeout_box.setMaximum(1000.)
        self.timeout_box.setSingleStep(.2)
        motion_box = QGroupBox('Motion', pbox)
        motion_layout = QFormLayout(motion_box)
        motion_box.setLayout(motion_layout)
        motion_layout.addRow('Speed', self.trans_vel_line)
        motion_layout.addRow('Speed (rotational)', self.rot_vel_line)
        motion_layout.addRow("&Arm", self.arm_radio_boxes)
        motion_layout.addRow('&Time Out', self.timeout_box)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)

        #formlayout.addRow(task_frame_box)
        formlayout.addRow('Frame', self.frame_box) 
        formlayout.addRow(motion_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])
        formlayout.addRow(self.pose_button)
        self.reset()

    def get_current_pose(self):
        frame_described_in = str(self.frame_box.currentText())
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'right':
            arm_tip_frame = PositionPriorityMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = PositionPriorityMoveTool.LEFT_TIP
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0))

        #print 'current pose', p_arm
        for value, vr in zip(p_arm[0], [self.xline, self.yline, self.zline]):
            vr.setValue(value)
        for value, vr in zip(tr.euler_from_quaternion(p_arm[1]), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setValue(np.degrees(value))

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
        timeout = self.timeout_box.value()
        pose_stamped = self.get_posestamped()
        return PositionPriorityState(nname, pose_stamped, trans_vel, rot_vel, arm, timeout)

    def set_node_properties(self, node):
        self.set_posestamped(node.pose_stamped)
        self.trans_vel_line.setValue(node.trans_vel)
        self.rot_vel_line.setValue(node.rot_vel)

        if node.arm == 'left':
            self.arm_radio_buttons[0].setChecked(True)
        if node.arm == 'right':
            self.arm_radio_buttons[1].setChecked(True)

        self.frame_box.setCurrentIndex(self.frame_box.findText(str(node.pose_stamped.header.frame_id)))
        self.timeout_box.setValue(node.timeout)

    def reset(self):
        self.frame_box.setCurrentIndex(self.frame_box.findText(self.default_frame))
        for vr in [self.xline, self.yline, self.zline, self.phi_line, self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.trans_vel_line.setValue(.02)
        self.rot_vel_line.setValue(.16)
        self.timeout_box.setValue(20)
        self.arm_radio_buttons[0].setChecked(True)

class PositionPriorityState(tu.StateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, pose_stamped, trans_vel, rot_vel, arm, #frame, 
            timeout):#, source_for_origin):
        tu.StateBase.__init__(self, name)
        self.pose_stamped  = pose_stamped
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel

        self.arm = arm
        #self.frame = frame
        self.timeout = timeout
        #self.set_remapping_for('origin', source_for_origin)

    def get_smach_state(self):
       return PositionPrioritySmach(self.pose_stamped, self.trans_vel, self.rot_vel,
                self.arm, 
                #self.frame, 
                self.timeout)#, self.remapping_for('origin'))

class PositionPrioritySmach(smach.State):

    def __init__(self, pose_stamped, trans_vel, rot_vel, arm, 
                #frame, 
                timeout):#, source_for_origin):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], 
                             input_keys = [], output_keys = [])

        self.pose_stamped  = pose_stamped
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel
        self.action_client = actionlib.SimpleActionClient(arm + '_ptp', ptp.LinearMovementAction)
        self.timeout = timeout

    def set_robot(self, robot_obj):
        if robot_obj != None:
            self.robot = robot_obj

    def execute_goal(self, goal, timeout):
        self.action_client.send_goal(goal)
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()

        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('PositionPrioritySmach: preempt requested')
                self.action_client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > timeout:
                self.action_client.cancel_goal()
                rospy.loginfo('PositionPrioritySmach: timed out!')
                succeeded = False
                break

            #print tu.goal_status_to_string(state)
            state = self.action_client.get_state()
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('PositionPrioritySmach: Succeeded!')
                    succeeded = True
                break

            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        return 'failed'

    def execute(self, userdata):
        goal = ptp.LinearMovementGoal()
        goal.goal = self.pose_stamped
        goal.trans_vel = self.trans_vel
        goal.rot_vel   = self.rot_vel
        return self.execute_goal(goal, self.timeout)

