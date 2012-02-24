import rcommander.tool_utils as tu
import tf_utils as tfu
import pr2_utils as p2u

from PyQt4.QtGui import *
from PyQt4.QtCore import *

import rospy
import geometry_msgs.msg as geo
import tf.transformations as tr
import smach
from object_manipulator.convert_functions import *

import numpy as np
import math
import threading

def pose_distance(ps_a, ps_b, tflistener):
    #put both into the same frame
    ps_a_fb = change_pose_stamped_frame(tflistener, ps_a, ps_b.header.frame_id)
    g_T_a = pose_to_mat(ps_a_fb.pose)
    g_T_b = pose_to_mat(ps_b.pose)

    a_T_b = g_T_a**-1 * g_T_b

    desired_trans = a_T_b[0:3, 3].copy()
    a_T_b[0:3, 3] = 0
    desired_angle, desired_axis, _ = tf.transformations.rotation_from_matrix(a_T_b)
    return desired_trans, desired_angle, desired_axis

def double_spin_box(parent, minv, maxv, step):
    box = QDoubleSpinBox(parent)
    box.setMinimum(minv)
    box.setMaximum(maxv)
    box.setSingleStep(step)
    return box

class SE3Tool:

    def source_changed(self, index):
        self.source_box.setCurrentIndex(index)
        source_name = str(self.source_box.currentText())
        if source_name != ' ':
            if source_name in self.posestamped_outputs:
                self.frame_box.setEnabled(False)
            else:
                self.frame_box.setEnabled(True)
        else:
            self.frame_box.setEnabled(True)

    def make_se3_boxes(self, pbox):
        self.xline = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
        self.yline = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
        self.zline = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
        self.phi_line   = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
        self.theta_line = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
        self.psi_line   = double_spin_box(pbox, -3.,3.,.01) #QLineEdit(pbox)
    
        position_box = QGroupBox('Position', pbox)
        position_layout = QFormLayout(position_box)
        position_box.setLayout(position_layout)
        position_layout.addRow("&X", self.xline)
        position_layout.addRow("&Y", self.yline)
        position_layout.addRow("&Z", self.zline)
    
        orientation_box = QGroupBox('Orientation', pbox)
        orientation_layout = QFormLayout(orientation_box)
        orientation_box.setLayout(orientation_layout)
        orientation_layout.addRow("&Phi",   self.phi_line)
        orientation_layout.addRow("&Theta", self.theta_line)
        orientation_layout.addRow("&Psi",   self.psi_line)
    
        return [position_box, orientation_box]
    
    def make_frame_box(self, pbox):
        frame_box = QComboBox(pbox)
        for f in self.tf_listener.getFrameStrings():
            frame_box.addItem(f)
        return frame_box
    
    def make_task_frame_box(self, pbox):
        self.frame_box = self.make_frame_box(pbox)
        self.posestamped_outputs  = self.rcommander.outputs_of_type(geo.PoseStamped)
        self.pointstamped_outputs = self.rcommander.outputs_of_type(geo.PointStamped)
        self.source_box = QComboBox(pbox)
        self.source_box.addItem(' ')
        for n in (self.posestamped_outputs + self.pointstamped_outputs):
            self.source_box.addItem(n)
        self.rcommander.connect(self.source_box, SIGNAL('currentIndexChanged(int)'), self.source_changed)
    
        task_frame_box = QGroupBox('Origin', pbox)
        task_frame_layout = QFormLayout(task_frame_box)
        task_frame_box.setLayout(task_frame_layout)
        task_frame_layout.addRow('&Frame', self.frame_box)
        task_frame_layout.addRow('&Source', self.source_box)
        return task_frame_box

    def get_pose(self):
        pose  = geo.Pose()
        pose.position = geo.Point(*[float(vr.value()) for vr in [self.xline, self.yline, self.zline]])
        pose.orientation = geo.Quaternion(*tr.quaternion_from_euler(*[float(vr.value()) for vr in [self.phi_line, self.theta_line, self.psi_line]]))
        return pose

    def set_pose(self, pose):
        for value, vr in zip(p2u.position(pose.position), [self.xline, self.yline, self.zline]):
            vr.setValue(value)
        for value, vr in zip(tr.euler_from_quaternion(p2u.quaternion(pose.orientation)), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setValue(value)

class VelocityPriorityMoveTool(tu.ToolBase, SE3Tool):

    LEFT_TIP = rospy.get_param('/l_cart/tip_name')
    RIGHT_TIP = rospy.get_param('/r_cart/tip_name')
    MIN_TIME = .01
    COMMAND_FRAME = '/torso_lift_link'

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'velocity_priority', 'Velocity Priority', VelocityPriorityState)
        self.default_frame = '/torso_lift_link'
        self.tf_listener = rcommander.tf_listener

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        task_frame_box = self.make_task_frame_box(pbox)
        group_boxes = self.make_se3_boxes(pbox)
        self.arm_radio_boxes, self.arm_radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'arm')
        self.list_manager = p2u.ListManager(self.get_current_data_cb, self.set_current_data_cb, None, name_preffix='point')
        list_widgets = self.list_manager.make_widgets(pbox, self.rcommander)
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        self.time_box = QDoubleSpinBox(pbox)
        #self.rcommander.connect(self.time_box, SIGNAL('valueChanged(double)'), self.time_box_value_changed_cb)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.1)

        formlayout.addRow(task_frame_box)
        formlayout.addRow('&Arm', self.arm_radio_boxes)
        formlayout.addRow('&Duration', self.time_box)

        for gb in group_boxes:
            formlayout.addRow(gb)

        formlayout.addRow(self.pose_button)

        for gb in list_widgets:
            formlayout.addRow(gb)
        self.reset()

    #def add_element_cb(self):
    #    d = self.list_manager.get_data()
    #    if len(d) == 0:
    #        return
    #    t = d[-1]['time'] + self.MIN_TIME
    #    self.time_box.setValue(t)

    #def time_box_value_changed_cb(self, v):
    #    #Don't let value be earlier than the last point in sery
    #    d = self.list_manager.get_data()
    #    if len(d) == 0:
    #        return
    #    t = d[-1]['time']
    #    if v <= t:
    #        v = t + self.MIN_TIME
    #    self.time_box.setValue(v)

    def get_current_pose(self):
        frame_described_in = str(self.frame_box.currentText())
        arm = p2u.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'right':
            arm_tip_frame = VelocityPriorityMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = VelocityPriorityMoveTool.LEFT_TIP
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = tfu.tf_as_matrix(self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0)))
        trans, rotation = tr.translation_from_matrix(p_arm), tr.quaternion_from_matrix(p_arm)
        for value, vr in zip(trans, [self.xline, self.yline, self.zline]):
            vr.setValue(value)
        for value, vr in zip(tr.euler_from_quaternion(rotation), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setValue(np.degrees(value))

    def get_current_data_cb(self):
        arm = p2u.selected_radio_button(self.arm_radio_buttons).lower()
        pose = self.get_pose()
        t = self.time_box.value()
        return {'arm': arm, 'pose': pose, 'time': t}

    def set_current_data_cb(self, data):
        if 'left' == data['arm']:
            self.arm_radio_buttons[0].setChecked(True)
        if data['arm'] == 'right':
            self.arm_radio_buttons[1].setChecked(True)
        self.set_pose(data['pose'])
        self.time_box.setValue(data['time'])

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        frame = str(self.frame_box.currentText())
        source_for_origin = str(self.source_box.currentText())
        if source_for_origin == ' ':
            source_for_origin = None

        poses_list = self.list_manager.get_data()
        return VelocityPriorityState(nname, frame, source_for_origin, poses_list)

    def set_node_properties(self, node):
        self.frame_box.setCurrentIndex(self.frame_box.findText(str(node.frame)))
        idx = p2u.combobox_idx(self.source_box, node.remapping_for('origin'))
        self.source_changed(idx)
        self.list_manager.set_data(node.poses_list)

    def reset(self):
        self.arm_radio_buttons[0].setChecked(True)
        for vr in [self.xline, self.yline, self.zline, self.phi_line, self.theta_line, self.psi_line]: 
            vr.setValue(0.0)
        self.frame_box.setCurrentIndex(self.frame_box.findText(self.default_frame))
        self.source_box.setCurrentIndex(self.source_box.findText(' '))
        self.time_box.setValue(0.5)

class VelocityPriorityState(tu.StateBase):

    def __init__(self, name, frame, source_for_origin, poses_list):
        tu.StateBase.__init__(self, name)
        self.frame = frame
        self.set_remapping_for('origin', source_for_origin)
        self.poses_list = poses_list
        self.robot = None

    def set_robot(self, robot_obj):
        self.robot = robot_obj

    def get_smach_state(self):
        return VelocityPriorityStateSmach(self.frame, self.remapping_for('origin'), self.poses_list, self.robot)

class PlayTrajectory(threading.Thread):

    def __init__(self, cart_pub, messages, CMD_T_frame):

        threading.Thread.__init__(self)
        self.cart_pub = cart_pub
        self.messages = messages
        self.CMD_T_frame = CMD_T_frame
        self.stop = False

    def set_stop(self):
        self.stop = True

    def run(self):
        #Change duration to cumulative time.
        timeslp = [0.0]
        for idx, messages[1:] in enumerate(el):
            timeslp.append(messages[idx]['time'] + timeslp[idx])

        wall_start_time = rospy.get_rostime().to_time()
        for t, el in zip(timeslp, messages):
            #sleep if needed
            cur_time = rospy.get_rostime().to_time()
            wall_time_from_start = cur_time - wall_start_time
            sleep_time = (t - wall_time_from_start) - .005
            if sleep_time > 0:
                time.sleep(sleep_time)

            if self.stop:
                return

            #Transform
            frame_T_p = tfu.tf_as_matrix((p2u.position(el['pose'].position), 
                                          p2u.quaternion(el['pose'].orientation)))
            CMD_T_p = self.CMD_T_frame * frame_T_p
            t, q = tfu.matrix_as_tf(CMD_T_p)
            command_pose = stamp_pose(mat_to_pose(CMD_T_p), VelocityPriorityMoveTool.COMMAND_FRAME)

            #Send
            self.cart_pub.publish(command_pose)

class VelocityPriorityStateSmach(smach.State):

    def __init__(self, frame, source_for_origin, poses_list, robot):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], 
                             input_keys = ['origin'], output_keys = [])
        self.frame = frame
        self.source_for_origin = source_for_origin
        self.poses_list = poses_list
        self.robot = robot

        self.lcart = rospy.Publisher('/l_cart/command_pose', geo.PoseStamped)
        self.rcart = rospy.Publisher('/r_cart/command_pose', geo.PoseStamped)

    def execute(self, userdata):
        CMD_T_frame = p2u.origin_to_frame(userdata.origin, self.frame, self.pr2.tf_listener, VelocityPriorityMoveTool.COMMAND_FRAME)
        #Sort into two lists
        lp = []
        rp = []
        for p in poses_list:
            if p['arm'] == 'left':
                lp.append(p)
            else:
                rp.append(p)
       
        lpthread = PlayTrajectory(self.lcart, lp, CMD_T_frame)
        lpthread.start()
        rpthread = PlayTrajectory(self.rcart, rp, CMD_T_frame)
        rpthread.start()

        r = rospy.Rate(100)
        preempted = False
        finished = False
        while not finished:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('VelocityPriorityStateSmach: preempt requested')
                #self.action_client.cancel_goal()
                lpthread.stop()
                self.service_preempt()
                preempted = True
                break

            if not lpthread.is_alive() and not rpthread.is_alive():
                #gripper_matrix = tfu.tf_as_matrix(self.robot.tf_listener.lookupTransform(
                #    VelocityPriorityMoveTool.COMMAND_FRAME, self.tool_frame, rospy.Time(0)))
                #gripper_ps = stamp_pose(mat_to_pose(gripper_matrix), VelocityPriorityMoveTool.COMMAND_FRAME)
                #pose_distance(gripper_ps, ,self.robot.tf_listener)

                finished = True
                #Check that
            r.sleep()

        if preempted:
            return 'preempted'

        if finished:
            return 'succeeded'

        return 'failed'














