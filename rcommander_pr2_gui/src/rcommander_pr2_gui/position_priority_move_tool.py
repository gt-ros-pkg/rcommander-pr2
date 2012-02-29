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


class PositionPriorityMoveTool(tu.ToolBase):

    #LEFT_TIP = 'l_gripper_tool_frame'
    #RIGHT_TIP = 'r_gripper_tool_frame'
    LEFT_TIP = rospy.get_param('/l_cart/tip_name')
    RIGHT_TIP = rospy.get_param('/r_cart/tip_name')

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'position_priority', 'Position Priority', PositionPriorityState)
        self.default_frame = '/torso_lift_link'
        self.tf_listener = rcommander.tf_listener
        #self.suggested_frames = ['/base_link', '/torso_lift_link', '/l_gripper_tool_frame', '/r_gripper_tool_frame']

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        #Make frame selection
        self.frame_box = QComboBox(pbox)
        for f in self.tf_listener.getFrameStrings():
            self.frame_box.addItem(f)

        #Make source point selection
        self.posestamped_outputs  = self.rcommander.outputs_of_type(geo.PoseStamped)
        self.pointstamped_outputs = self.rcommander.outputs_of_type(geo.PointStamped)
        self.source_box = QComboBox(pbox)
        self.source_box.addItem(' ')
        for n in (self.posestamped_outputs + self.pointstamped_outputs):
            self.source_box.addItem(n)
        self.rcommander.connect(self.source_box, SIGNAL('currentIndexChanged(int)'),    self.source_changed)

        task_frame_box = QGroupBox('Origin', pbox)
        task_frame_layout = QFormLayout(task_frame_box)
        task_frame_box.setLayout(task_frame_layout)
        task_frame_layout.addRow('&Frame', self.frame_box)
        task_frame_layout.addRow('&Source', self.source_box)

        #Group Position
        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)
        self.trans_vel_line = QLineEdit(pbox)
        position_box = QGroupBox('Position', pbox)
        position_layout = QFormLayout(position_box)
        position_box.setLayout(position_layout)
        position_layout.addRow("&X", self.xline)
        position_layout.addRow("&Y", self.yline)
        position_layout.addRow("&Z", self.zline)
        position_layout.addRow('&Velocity', self.trans_vel_line)

        #Group Orientation
        self.phi_line   = QLineEdit(pbox)
        self.theta_line = QLineEdit(pbox)
        self.psi_line   = QLineEdit(pbox)
        self.rot_vel_line   = QLineEdit(pbox)
        orientation_box = QGroupBox('Orientation', pbox)
        orientation_layout = QFormLayout(orientation_box)
        orientation_box.setLayout(orientation_layout)
        
        orientation_layout.addRow("&Phi",   self.phi_line)
        orientation_layout.addRow("&Theta", self.theta_line)
        orientation_layout.addRow("&Psi",   self.psi_line)
        orientation_layout.addRow('&Velocity', self.rot_vel_line)

        self.arm_radio_boxes, self.arm_radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'arm')
        self.timeout_box = QDoubleSpinBox(pbox)
        self.timeout_box.setMinimum(0)
        self.timeout_box.setMaximum(1000.)
        self.timeout_box.setSingleStep(.2)
        motion_box = QGroupBox('Motion', pbox)
        motion_layout = QFormLayout(motion_box)
        motion_box.setLayout(motion_layout)
        motion_layout.addRow("&Arm", self.arm_radio_boxes)
        motion_layout.addRow('&Time Out', self.timeout_box)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)

        formlayout.addRow(task_frame_box)
        formlayout.addRow(position_box)
        formlayout.addRow(orientation_box)
        formlayout.addRow(motion_box)
        formlayout.addRow(self.pose_button)
        self.reset()

    def source_changed(self, index):
        self.source_box.setCurrentIndex(index)
        source_name = str(self.source_box.currentText())
        if source_name != ' ':
            #If the thing selected is a pose stamped, disable frame box!
            if source_name in self.posestamped_outputs:
                self.frame_box.setEnabled(False)
            else:
                self.frame_box.setEnabled(True)
        else:
            self.frame_box.setEnabled(True)

    def get_current_pose(self):
        frame_described_in = str(self.frame_box.currentText())
        arm = p2u.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'right':
            arm_tip_frame = PositionPriorityMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = PositionPriorityMoveTool.LEFT_TIP
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0))

        #print 'CURRENT POSE TRANSFORM', p_arm
        #p_arm = tfu.tf_as_matrix(frame_T_tip)
        #trans, rotation = tr.translation_from_matrix(p_arm), tr.quaternion_from_matrix(p_arm)
        #print 'TRANS', trans
        #print 'ROTATION', rotation
        #for value, vr in zip(tr.euler_from_quaternion(rotation), [self.phi_line, self.theta_line, self.psi_line]):
        #print 'EULER', tr.euler_from_quaternion(p_arm[1])

        for value, vr in zip(p_arm[0], [self.xline, self.yline, self.zline]):
            vr.setText("%.3f" % value)
        for value, vr in zip(tr.euler_from_quaternion(p_arm[1]), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText("%.3f" % np.degrees(value))

        #print 'quat from euler!', tr.quaternion_from_euler(np.radians(float(self.phi_line.text())), 
        #                                                   np.radians(float(self.theta_line.text())),
        #                                                   np.radians(float(self.psi_line.text())))
        #        #*[float(vr.text()) for vr in [self.phi_line, self.theta_line, self.psi_line]])


    def new_node(self, name=None):
        #make name
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        #make pose
        pose  = geo.Pose()
        pose.position = geo.Point(*[float(vr.text()) for vr in [self.xline, self.yline, self.zline]])
        pose.orientation = geo.Quaternion(*tr.quaternion_from_euler(*[np.radians(float(vr.text())) for vr in [self.phi_line, self.theta_line, self.psi_line]]))

        #print 'NEW NODE POSE', pose

        #other properties
        trans_vel = float(str(self.trans_vel_line.text()))
        rot_vel   = float(str(self.rot_vel_line.text()))
        arm = p2u.selected_radio_button(self.arm_radio_buttons).lower()
        frame  = str(self.frame_box.currentText())
        timeout = self.timeout_box.value()
        source_for_origin = str(self.source_box.currentText())
        if source_for_origin == ' ':
            source_for_origin = None

        return PositionPriorityState(nname,
                pose, trans_vel, rot_vel, 
                arm, frame, timeout, source_for_origin)


    def set_node_properties(self, node):
        #node.pose
        for value, vr in zip(p2u.position(node.pose.position), [self.xline, self.yline, self.zline]):
            vr.setText(str(value))
        for value, vr in zip(tr.euler_from_quaternion(p2u.quaternion(node.pose.orientation)), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText(str(value))

        self.trans_vel_line.setText(str(node.trans_vel))
        self.rot_vel_line.setText(str(node.rot_vel))

        if node.arm == 'left':
            self.arm_radio_buttons[0].setChecked(True)
        if node.arm == 'right':
            self.arm_radio_buttons[1].setChecked(True)

        self.frame_box.setCurrentIndex(self.frame_box.findText(str(node.frame)))
        self.timeout_box.setValue(node.timeout)

        #Set sources
        idx = p2u.combobox_idx(self.source_box, node.remapping_for('origin'))
        self.source_changed(idx)

    def reset(self):
        self.frame_box.setCurrentIndex(self.frame_box.findText(self.default_frame))
        self.source_box.setCurrentIndex(self.source_box.findText(' '))
        for vr in [self.xline, self.yline, self.zline, self.phi_line, self.theta_line, self.psi_line]:
            vr.setText(str(0.0))
        self.trans_vel_line.setText(str(.02))
        self.rot_vel_line.setText(str(.16))
        self.timeout_box.setValue(20)
        self.arm_radio_buttons[0].setChecked(True)


class PositionPriorityState(tu.StateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, pose, trans_vel, rot_vel, 
            arm, frame, timeout, source_for_origin):
        tu.StateBase.__init__(self, name)
        self.pose  = pose
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel

        self.arm = arm
        self.frame = frame
        self.timeout = timeout
        self.set_remapping_for('origin', source_for_origin)

    def get_smach_state(self):
       return PositionPrioritySmach(self.pose, self.trans_vel, self.rot_vel,
                self.arm, self.frame, self.timeout, self.remapping_for('origin'))

def origin_to_frame(origin, supplement_frame, tf_listener, command_frame):
    m = origin
    if isinstance(m, geo.PointStamped):
        #point in some frame, needs orientation...
        #print 'command_frame', command_frame, 'supplement_frame', supplement_frame, 'header_frame', m.header.frame_id
        command_frame_T_supplement_frame = tfu.tf_as_matrix(tf_listener.lookupTransform(command_frame, supplement_frame, rospy.Time(0)))
        #print 'command_frame_T_supplement_frame\n', command_frame_T_supplement_frame
        command_frame_T_header_frame = tfu.tf_as_matrix(tf_listener.lookupTransform(command_frame, m.header.frame_id, rospy.Time(0)))
        #print 'command_frame_T_header_frame\n', command_frame_T_header_frame
        point_header = np.matrix([m.point.x, m.point.y, m.point.z, 1.]).T
        point_command_frame = command_frame_T_header_frame * point_header
        #print 'point header\n', point_header.T
        #print 'point_command_frame\n', point_command_frame.T

        command_frame_T_point_frame = command_frame_T_supplement_frame.copy()
        command_frame_T_point_frame[0:3,3] = point_command_frame[0:3,0]
        CMD_T_frame = command_frame_T_point_frame
        #print 'command_frame_T_point_frame\n', command_frame_T_point_frame

        #print 'got PointStamped\n', m
        #CMD_T_pf = tfu.tf_as_matrix(tf_listener.lookupTransform(command_frame, m.header.frame_id, rospy.Time(0)))
        #print 'FRAMES', command_frame, m.header.frame_id
        #print 'CMD_T_pf\n', CMD_T_pf
        #p_CMD = CMD_T_pf * np.matrix([m.point.x, m.point.y, m.point.z, 1.]).T
        #print 'p_CMD\n', p_CMD
        #CMD_T_f = tfu.tf_as_matrix(tf_listener.lookupTransform(command_frame, supplement_frame, rospy.Time(0)))
        #print 'CMD_T_f\n', CMD_T_f
        #CMD_T_frame = tll_T_f.copy()
        #CMD_T_frame[0:3, 3] = p_CMD[0:3, 0]
        #print 'CMD_T_frame\n', CMD_T_frame

    #If it's a pose stamped then we turn the pose stamped into a frame?
    elif isinstance(m, geo.PoseStamped):
        fid_T_p = pose_to_mat(m.pose)
        print 'fid_T_p\n', fid_T_p
        print fid_T_p[0:3,3].T
        CMD_T_fid = tfu.tf_as_matrix(tf_listener.lookupTransform(command_frame, m.header.frame_id, rospy.Time(0)))
        print 'FRAMES', command_frame, m.header.frame_id
        print 'CMD_T_fid\n', CMD_T_fid
        print CMD_T_fid[0:3,3].T
        CMD_T_frame = CMD_T_fid * fid_T_p
        print 'CMD_T_frame\n', CMD_T_frame
        print CMD_T_frame[0:3,3].T
    else:
        raise RuntimeError('Got origin that is an instance of ' + str(m.__class__))

    return CMD_T_frame


class PositionPrioritySmach(smach.State):

    def __init__(self, pose, trans_vel, rot_vel, arm, frame, timeout, source_for_origin):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], 
                             input_keys = ['origin'], output_keys = [])
        self.COMMAND_FRAME = '/torso_lift_link'

        self.pose  = pose
        self.trans_vel = trans_vel
        self.rot_vel = rot_vel

        self.action_client = actionlib.SimpleActionClient(arm + '_ptp', ptp.LinearMovementAction)
        self.frame = frame
        self.timeout = timeout
        self.source_for_origin = source_for_origin

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
        #Need to figure out the point as expressed in its new target frame as
        #the target frame might not be in TF 
        if self.source_for_origin != None:
            #Frame is not in TF so call helper
            CMD_T_frame = origin_to_frame(userdata.origin, self.frame, self.robot.tf_listener, self.COMMAND_FRAME)
        else:
            #Frame is in TF so everything is OK
            CMD_T_frame = tfu.tf_as_matrix(self.robot.tf_listener.lookupTransform(self.COMMAND_FRAME, self.frame, rospy.Time(0)))

        goal = ptp.LinearMovementGoal()
        goal.goal      = stamp_pose(mat_to_pose(CMD_T_frame * pose_to_mat(self.pose)), self.COMMAND_FRAME)
        goal.trans_vel = self.trans_vel
        goal.rot_vel   = self.rot_vel
        #print 'self.pose', self.pose
        #print 'GOAL!', goal

        return self.execute_goal(goal, self.timeout)

