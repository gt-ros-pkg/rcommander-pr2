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
            #self.xline.setEnabled(False)
            #self.yline.setEnabled(False)
            #self.zline.setEnabled(False)
            #self.motion_box_position.setCurrentIndex(self.motion_box_position.findText('absolute'))
            #self.motion_box_position.setEnabled(False)
        else:
            self.frame_box.setEnabled(True)
            #self.xline.setEnabled(True)
            #self.yline.setEnabled(True)
            #self.zline.setEnabled(True)
            #self.motion_box_position.setEnabled(True)

    #def source_changed_orientation(self, index):
    #    self.source_box_orientation.setCurrentIndex(index)
    #    if str(self.source_box_orientation.currentText()) != ' ':
    #        self.phi_line.setEnabled(False)
    #        self.theta_line.setEnabled(False)
    #        self.psi_line.setEnabled(False)
    #        #self.motion_box_orientation.setCurrentIndex(self.motion_box_orientation.findText('absolute'))
    #        #self.motion_box_orientation.setEnabled(False)
    #    else:
    #        self.phi_line.setEnabled(True)
    #        self.theta_line.setEnabled(True)
    #        self.psi_line.setEnabled(True)
    #        #self.motion_box_orientation.setEnabled(True) 

    def get_current_pose(self):
        frame_described_in = str(self.frame_box.currentText())
        arm = p2u.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'right':
            arm_tip_frame = PositionPriorityMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = PositionPriorityMoveTool.LEFT_TIP
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = tfu.tf_as_matrix(self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0)))
        trans, rotation = tr.translation_from_matrix(p_arm), tr.quaternion_from_matrix(p_arm)
        for value, vr in zip(trans, [self.xline, self.yline, self.zline]):
            vr.setText("%.3f" % value)
        for value, vr in zip(tr.euler_from_quaternion(rotation), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText("%.3f" % np.degrees(value))


    def new_node(self, name=None):
        #make name
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        #make pose
        pose  = geo.Pose()
        pose.position = geo.Point(*[float(vr.text()) for vr in [self.xline, self.yline, self.zline]])
        pose.orientation = geo.Quaternion(*tr.quaternion_from_euler(*[float(vr.text()) for vr in [self.phi_line, self.theta_line, self.psi_line]]))

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

        #self.motion_box_position.setCurrentIndex(self.motion_box_position.findText('relative'))
        #self.motion_box_orientation.setCurrentIndex(self.motion_box_orientation.findText('relative'))
        #self.source_box_orientation.setCurrentIndex(self.source_box_orientation.findText(' '))
        #self.source_box_position.setCurrentIndex(self.source_box_position.findText(' '))
        #self.arm_box.setCurrentIndex(self.arm_box.findText('left'))


    ##
    # @param name
    # @param trans list of 3 floats
    # @param angles in euler list of 3 floats
    # @param frame 
    #def __init__(self, name, arm,
    #    trans,  motion_type_trans,  source_trans,
    #    angles, motion_type_angles, source_angles,
    #    vels, frame, timeout):

    #    tu.StateBase.__init__(self, name)
    #    self.arm = arm

    #    self.trans = trans
    #    self.motion_type_trans = motion_type_trans
    #    self.set_remapping_for('position', source_trans)

    #    self.set_angles(angles)
    #    self.motion_type_angles = motion_type_angles
    #    self.set_remapping_for('orientation', source_angles)

    #    self.vels = vels
    #    self.frame = frame
    #    self.timeout = timeout
    #    #self.angles = angles #convert angles to _quat

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

    #def set_angles(self, euler_angs):
    #    ang_rad = [np.radians(e) for e in euler_angs]
    #    self.quat = tr.quaternion_from_euler(*ang_rad)
    #    #self._quat = tr.quaternion_from_euler(euler_angs[0], euler_angs[1], euler_angs[2])
    #
    #def get_angles(self):
    #    return [np.degrees(e) for e in tr.euler_from_quaternion(self.quat)]
    ##angles = property(_get_angles, _set_angles)

    def get_smach_state(self):
        #return LinearMovementSmach(self.arm,
        #          self.trans, self.motion_type_trans, self.remapping_for('position'),
        #          self.quat, self.motion_type_angles, self.remapping_for('orientation'),
        #          self.vels, self.frame, self.timeout)
        return PositionPrioritySmach(self.pose, self.trans_vel, self.rot_vel,
                self.arm, self.frame, self.timeout, self.remapping_for('origin'))

        #return LinearMovementSmach(motion_type = self.motion_type, arm = self.arm, trans = self.trans, 
        #        quat = self.quat, frame = self.frame, vels = self.vels, 
        #        source_for_point = self.remapping_for('point'), timeout=self.timeout)


def origin_to_frame(origin, supplement_frame, tf_listener, command_frame):
    m = origin
    if instanceof(m, geo.PointStamped):
        #point in some frame, needs orientation...
        CMD_T_pf = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
        p_CMD = CMD_T_pf * np.matrix([m.x, m.y, m.z, 1.]).T
        CMD_T_f = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, supplement_frame, rospy.Time(0)))
        CMD_T_frame = tll_T_f.copy()
        CMD_T_frame[0:3, 3] = p_CMD[0:3, 0]
    #If it's a pose stamped then we turn the pose stamped into a frame?
    elif instanceof(m, geo.PoseStamped):
        fid_T_p = pose_to_mat(m.pose)
        CMD_T_fid = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
        CMD_T_frame = CMD_T_fid * fid_T_p
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
            CMD_T_frame = origin_to_frame(userdata.origin, self.frame, self.pr2.tf_listener, self.COMMAND_FRAME)
        else:
            #Frame is in TF so everything is OK
            CMD_T_frame = tfu.tf_as_matrix(self.pr2.tf_listener.lookupTransform(self.COMMAND_FRAME, self.frame, rospy.Time(0)))

        goal = ptp.LinearMovementGoal()
        goal.goal      = stamp_pose(mat_to_pose(CMD_T_frame * pose_to_mat(self.pose)), self.COMMAND_FRAME)
        goal.trans_vel = self.trans_vel
        goal.rot_vel   = self.rot_vel

        return self.execute_goal(goal, self.timeout)




#def make_radio_box(parent, options, name_preffix):
#    container_name = name_preffix + '_radio_box'
#
#    container = qtg.QWidget(parent)
#    container.setObjectName(container_name)
#    hlayout = qtg.QHBoxLayout(container)
#    radio_buttons = []
#
#    for option in options:
#        r = qtg.QRadioButton(container)
#        r.setObjectName(name_preffix + '_' + option)
#        r.setText(option)
#        hlayout.addWidget(r)
#        radio_buttons.append(r)
#    radio_buttons[0].setChecked(True)
#
#    return container, radio_buttons



#class LinearMovementSmach(smach.State):
#
#    def __init__(self, arm):
#
#    def set_robot(self, robot):
#        self.pr2 = robot
#
#    def execute(self, userdata):
#        self.source_for
#        self.pr2
#        self.frame
#        userdata.message
#        self.poses = {'timeout':p, 'trans_vel':p, 'rot_vel'p}
#
#        COMMAND_FRAME = '/torso_lift_link'
#        # Look up inputs if they exist and grab data
#        if self.source_for != None:
#            m = userdata.message
#            #If it's a point stamped then we create a frame.
#            if instanceof(m, geo.PointStamped):
#                #point in some frame, needs orientation...
#                CMD_T_pf = tr.tf_as_matrix(self.pr2.tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
#                p_CMD = CMD_T_pf * np.matrix([m.x, m.y, m.z, 1.]).T
#                CMD_T_f = tr.tf_as_matrix(self.pr2.tf_listener.lookupTransform(COMMAND_FRAME, self.frame, rospy.Time(0)))
#                CMD_T_frame = tll_T_f.copy()
#                CMD_T_frame[0:3, 3] = p_CMD[0:3, 0]
#            #If it's a pose stamped then we turn the pose stamped into a frame?
#            elif instanceof(m, geo.PoseStamped):
#                fid_T_p = pose_to_mat(m.pose)
#                CMD_T_fid = tr.tf_as_matrix(self.pr2.tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
#                CMD_T_frame = CMD_T_fid * fid_T_p
#            else:
#                raise RuntimeError('Got message that is an instance of ' + str(m.__class__))
#        else:
#            CMD_T_frame = tr.tf_as_matrix(self.pr2.tf_listener.lookupTransform(COMMAND_FRAME, self.frame, rospy.Time(0)))
#
#        # For each point in trajectory (expressed in given frame), reexpress it in TLL then
#        # If we're sending this to the action server (which tries to guarantee success)
#        # then send the point.  If not, send to *_cart
#        # Look up the frame we need to transform point to: CMD_T_frame
#        for p in self.poses:
#            p_CMD = CMD_T_frame * pose_to_mat(p['pose'])
#            timeout = p['timeout']
#            #SEND command, wait for timeout
#            goal = ptp.LinearMovementGoal()
#            goal.goal      = stamp_pose(mat_to_pose(p_CMD), COMMAND_FRAME)
#            goal.trans_vel = p['trans_vel']
#            goal.rot_vel   = p['rot_vel']
#            self.action_client.send_goal(goal)
#
#    ##
#    # failure and success depends on the *last goal* in trajectory
#    def execute(self, userdata):
#        r = rospy.Rate(30)
#        ret = 'failed'
#        for goal, timeout in self.ros_goal(userdata):
#            ret = self.execute_goal(goal, timeout)
#            if ret == 'preempted':
#                return ret
#        return ret
#
#    def execute_goal(self, goal, timeout):
#        self.action_client.send_goal(goal)
#        succeeded = False
#        preempted = False
#        r = rospy.Rate(30)
#        start_time = rospy.get_time()
#
#        while True:
#            #we have been preempted
#            if self.preempt_requested():
#                rospy.loginfo('LinearMoveStateSmach: preempt requested')
#                self.action_client.cancel_goal()
#                self.service_preempt()
#                preempted = True
#                break
#
#            if (rospy.get_time() - start_time) > timeout:
#                self.action_client.cancel_goal()
#                rospy.loginfo('LinearMoveStateSmach: timed out!')
#                succeeded = False
#                break
#
#            #print tu.goal_status_to_string(state)
#            state = self.action_client.get_state()
#            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
#                if state == am.GoalStatus.SUCCEEDED:
#                    rospy.loginfo('LinearMoveStateSmach: Succeeded!')
#                    succeeded = True
#                break
#
#            r.sleep()
#
#        if preempted:
#            return 'preempted'
#
#        if succeeded:
#            return 'succeeded'
#
#        return 'failed'

















        #action: point sent to ptp_action commands the TIP of the gripper.
        #*_cart: point sent controls the WRIST (so we'll need to do some adjustments)





        #Our input is either a frame or a PoseStamped or a PointStamped

        #trans = self.trans
        #if self.source_for_trans != None:
        #    p = userdata.position.pose.position
        #    trans = [p.x, p.y, p.z]

        #quat = self.quat
        #if self.source_for_trans != None:
        #    q = userdata.orientation.pose.orientation
        #    quat = [q.x, q.y, q.z, q.w]

        # If data is relative, grab the frame and transform them
       #   Four cases:
        #    Relative position, relative orientation
        #    Absolute position, relative orientation
        #    Relative position, absolute orientation
        #    Absolute position, Absolute orientation

        #if self.motion_type_trans == 'relative':

        #if self.motion_type_trans == 'relative':

        #if self.motion_type_angles == 'absolute':

        #if self.motion_type_angles == 'relative':

        #if self.source_for_trans != None:
        #    p = userdata.point.pose.position
        #    q = userdata.point.pose.orientation
        #    trans = [p.x, p.y, p.z]
        #    quat = [q.x, q.y, q.z, q.w]
        #    frame = userdata.point.header.frame_id
        #    quat = self.pr2.tf_listener.lookupTransform(frame, tip, rospy.Time(0))[1]
        #else:
        #    trans = self.trans
        #    frame = self.frame
        #    quat = self.quat

        # pose = mat_to_pose(np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(quat)))
        # goal.goal = stamp_pose(pose, frame)
        # goal.trans_vel = self.vels[0]
        # goal.rot_vel = self.vels[1]
        # return goal

        ## Send goal with newly calculated points

        #    rospy.loginfo('Received relative motion.')

        #    #print 'tool frame is', self.tool_frame
        #    #print 'goal frame is', goal_ps.header.frame_id

        #    delta_ref  = pose_to_mat(goal_ps.pose)
        #    tll_R_ref = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', goal_ps.header.frame_
        #    tll_R_ref[0:3,3] = 0
        #    delta_tll = tll_R_ref * delta_ref

        #    #print 'delta_tll\n', delta_tll
        #    tip_current_T_tll = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', self.tool_fra
        #    #print 'tip_current_T_tll\n', tip_current_T_tll

        #    #Find translation
        #    delta_T = delta_tll.copy()
        #    delta_T[0:3,0:3] = np.eye(3)
        #    tip_T = delta_T * tip_current_T_tll

        #    #Find rotation
        #    tip_R = delta_tll[0:3, 0:3] * tip_current_T_tll[0:3, 0:3]


        #    tip_new = np.matrix(np.eye(4))
        #    tip_new[0:3, 0:3] = tip_R
        #    tip_new[0:3, 3] = tip_T[0:3,3]

        #    #print 'tip_new\n', tip_new
        #    goal_ps = stamp_pose(mat_to_pose(tip_new), 'torso_lift_link')

        ##if self.motion_type == 'relative':
        ##    #goal.relative = True
        ##elif self.motion_type == 'absolute':
        ##    #goal.relative = False
        ##else:
        ##    raise RuntimeError('Invalid motion type given.')




#class ListModel(QAbstractItemModel):
#
#    def index(self, row, column, qmodel_idx_parent):
#        pass
#        
#    def parent(self, qmodel_idx):
#        pass 
#
#    def rowCount(self, qmodel_idx):
#        pass
#
#    def columnCount(self, qmodel_idx):
#        pass
#        
#    def data(self, qmodel_idx, disp_role):
#        pass
#
#    def insertRows(self, row, count, qmodel_idx):
#        pass
#
#    def insertRows(self, row, count, qmodel_idx):
#        pass

            #if self.arm == 'left':
            #    tip = rospy.get_param('/l_cart/tip_name')
            #    tool = 'l_gripper_tool_frame'
            #if self.arm == 'right':
            #    tip = rospy.get_param('/r_cart/tip_name')
            #    tool = 'r_gripper_tool_frame'

            #print 'point before', trans
            #tip_T_tool = tr.tf_as_matrix(self.pr2.tf_listener.lookupTransform(tip, wrist, rospy.Time(0)))
            #point_tip = tip_T_tool * tr.tf_as_matrix((trans,quat))
            #trans, quat = matrix_as_tf(point_tip)
            #print 'point after', trans

##
## name maps to tool used to create it
## model
## is a state that can be stuffed into a state machine
#class LinearMoveState(tu.SimpleStateBase): # smach_ros.SimpleActionState):
#    ##
#    #
#    # @param name
#    # @param trans list of 3 floats
#    # @param angles in euler list of 3 floats
#    # @param frame 
#    def __init__(self, name, trans, angles, arm, vels, motion_type, source, frame):
#        tu.SimpleStateBase.__init__(self, name, \
#                arm + '_ptp', ptp.LinearMovementAction, 
#                goal_cb_str = 'ros_goal', input_keys=['point']) 
#        self.set_remapping_for('point', source)
#        #self.register_input_keys(['point'])
#        #print 'registered input keys', self.get_registered_input_keys()
#
#        self.trans = trans
#        self.angles = angles #convert angles to _quat
#        self.arm = arm
#        self.vels = vels
#        self.motion_type = motion_type
#        self.frame = frame
#
#    def set_robot(self, pr2):
#        self.pr2 = pr2
#
#    def get_smach_state(self):
#        state = tu.SimpleStateBase.get_smach_state(self)
#        state.set_robot = self.
#
#    def ros_goal(self, userdata, default_goal):
#        #print 'LinearMoveState: rosgoal called!!!!!!!!!!!!!!1'
#        goal = ptp.LinearMovementGoal()
#        if self.motion_type == 'relative':
#            goal.relative = True
#        elif self.motion_type == 'absolute':
#            goal.relative = False
#        else:
#            raise RuntimeError('Invalid motion type given.')
#
#        quat = self._quat
#        if self.source_for('point') != None:
#            trans, frame = userdata.point
#            if self.arm == 'left':
#                tip = rospy.get_param('/l_cart/tip_name')
#            if self.arm == 'right':
#                tip = rospy.get_param('/r_cart/tip_name')
#            quat = self.pr2.tf_listener.lookupTransform(frame, tip, rospy.Time(0))[1]
#        else:
#            trans = self.trans
#            frame = self.frame
#
#        pose = mat_to_pose(np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(quat)))
#        goal.goal = stamp_pose(pose, frame)
#        goal.trans_vel = self.vels[0]
#        goal.rot_vel = self.vels[1]
#        #print 'returned goal'
#        return goal
#
#    def _set_angles(self, euler_angs):
#        ang_rad = [np.radians(e) for e in euler_angs]
#        #self._quat = tr.quaternion_from_euler(euler_angs[0], euler_angs[1], euler_angs[2])
#        self._quat = tr.quaternion_from_euler(*ang_rad)
#    
#    def _get_angles(self):
#        return [np.degrees(e) for e in tr.euler_from_quaternion(self._quat)]
#        
#    angles = property(_get_angles, _set_angles)
#
#    #def __getstate__(self):
#    #    state = tu.SimpleStateBase.__getstate__(self)
#    #    my_state = [self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame]
#    #    return {'simple_state': state, 'self': my_state}
#
#    #def __setstate__(self, state):
#    #    tu.SimpleStateBase.__setstate__(self, state['simple_state'])
#    #    self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame = state['self']
#
#class SimpleStateBaseSmach(smach_ros.SimpleActionState):
#
#    def __init__(self, action_name, action_type, goal_obj, goal_cb_str, input_keys, output_keys):
#        smach_ros.SimpleActionState.__init__(self, action_name, action_type, 
#                goal_cb = SimpleStateCB(eval('goal_obj.%s' % goal_cb_str), input_keys, output_keys))
#        self.goal_obj = goal_obj
#
#    def __call__(self, userdata, default_goal): 
#        f = eval('self.goal_obj.%s' % self.goal_cb_str)
#        return f(userdata, default_goal)
#
#===============================================================================
#===============================================================================
#===============================================================================
#===============================================================================
