#import roslib; roslib.load_manifest('rcommander_pr2')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import numpy as np
import actionlib_msgs.msg as am
import smach
import functools as ft
import pr2_utils as pu
from rcommander_pr2_gui.srv import MinTime
import trajectory_msgs.msg as tm
import os.path as pt
import roslib

def create_color(a, r,g,b):
    palette = QPalette(QColor(a, r, g, b))
    palette.setColor(QPalette.Text, QColor(a, r, g, b))
    return palette

class JointSequenceTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'joint_sequence', 'Joint Sequence', JointSequenceState)
        self.joint_name_fields = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint", 
                                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.reverse_idx = {}
        for idx, n in enumerate(self.joint_name_fields):
            self.reverse_idx[n] = idx

        self.joint_angs_list = None

        self.status_bar_timer = QTimer()
        self.rcommander.connect(self.status_bar_timer, SIGNAL('timeout()'), self.get_current_joint_angles)
        self.limits = [self.rcommander.robot.left.get_limits(), self.rcommander.robot.right.get_limits()]
        self.min_time_service = rospy.ServiceProxy('min_time_to_move', MinTime)
        #self.vel_limits = [self.rcommander.robot.left.get_vel_limits(), self.rcommander.robot.right.get_vel_limits()]
        self.current_update_color = create_color(0,0,0,255)

    def _value_changed_validate(self, value, joint):
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        #limits = self.limits[idx]
        #jname = pref + joint
        #if limits.has_key(jname):
        #    exec('box = self.%s' % joint)
        #    mina, maxa = limits[jname]
        #    v = np.radians(value)
        #    if v < mina or v > maxa:
        #        self.set_invalid_color(joint, True)
        #    else:
        #        self.set_invalid_color(joint, False)
        self._check_limit(arm, value, joint)
        self._check_time_validity(self.time_box.value())

    def _check_limit(self, arm, value, joint):
        if arm == 'left':
            idx = 0
            pref = 'l_'
        else:
            idx = 1
            pref = 'r_'

        limits = self.limits[idx]
        jname = pref + joint
        if limits.has_key(jname):
            exec('box = self.%s' % joint)
            mina, maxa = limits[jname]
            v = np.radians(value)
            if v < mina or v > maxa:
                self.set_invalid_color(joint, True)
            else:
                self.set_invalid_color(joint, False)

    def _check_time_validity(self, value):
        #self.set_invalid_color('time_box', False)
        r,g,b = 0,0,0
        palette = QPalette(QColor(r, g, b, 255))
        palette.setColor(QPalette.Text, QColor(r, g, b, 255))
        self.time_box.setPalette(palette)

        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'left':
            #idx = 0
            pref = 'l_'
            left_arm = True
        else:
            #idx = 1
            pref = 'r_'
            left_arm = False

        # vel_limits = self.vel_limits[idx]
        if len(self.joint_angs_list) == 0:
            return

        if self.curr_selected == None:
            ref = self.joint_angs_list[-1]
        else:
            pidx = self.curr_selected - 1
            if pidx < 0:
                return 
            else:
                ref = self.joint_angs_list[pidx]

        start_point = tm.JointTrajectoryPoint()
        start_point.velocities = [0.] * len(self.joint_name_fields)
        for name in self.joint_name_fields:
            exec('box = self.%s' % name)
            start_point.positions.append(np.radians(box.value()))

        end_point = tm.JointTrajectoryPoint()
        end_point.velocities = [0.] * len(self.joint_name_fields)
        end_point.positions = ref['angs']

        min_time = self.min_time_service(start_point, end_point, left_arm).time
        curr_time = self.time_box.value()
        #print 'min_time', min_time
        for multiplier, color in [[1., [255,0,0]], [2., [255,153,0]]]:
            if curr_time <= (min_time * multiplier):
                r,g,b = color
                palette = QPalette(QColor(r, g, b, 255))
                palette.setColor(QPalette.Text, QColor(r, g, b, 255))
                self.time_box.setPalette(palette)
                #self.set_invalid_color('time_box', True, color)
                return

    def _time_changed_validate(self, value):
        self._check_time_validity(value)

    def set_invalid_color(self, joint_name, invalid, color = [255,0,0]):
        r,g,b = color

        if invalid:
            palette = QPalette(QColor(r, g, b, 255))
            palette.setColor(QPalette.Text, QColor(r, g, b, 255))
        else:
            #exec('self.%s.setPalette(palette)' % name)
            palette = self.current_update_color
            #palette = QPalette(QColor(0, 0, 0, 255))
            #palette.setColor(QPalette.Text, QColor(0, 0, 0, 255))

        exec('self.%s.setPalette(palette)' % joint_name)

    def fill_property_box(self, pbox):
        self.curr_selected = None
        formlayout = pbox.layout()

        self.arm_radio_boxes, self.arm_radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'arm')
        formlayout.addRow('&Arm', self.arm_radio_boxes)

        #Controls for displaying the current joint states
        for name in self.joint_name_fields:
            #exec("self.%s = QLineEdit(pbox)" % name)
            exec("self.%s = QDoubleSpinBox(pbox)" % name)
            exec('box = self.%s' % name)
            box.setSingleStep(.5)
            box.setMinimum(-9999999)
            box.setMaximum(9999999)
            formlayout.addRow("&%s" % name, box)
            vchanged_func = ft.partial(self._value_changed_validate, joint=name)
            self.rcommander.connect(box, SIGNAL('valueChanged(double)'), vchanged_func)

        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.2)
        self.time_box.setValue(1.)
        formlayout.addRow('&Time', self.time_box)
        self.rcommander.connect(self.time_box, SIGNAL('valueChanged(double)'), self._time_changed_validate)
    
        self.live_update_button = QPushButton(pbox)
        self.live_update_button.setText('Live Update')
        self.rcommander.connect(self.live_update_button, SIGNAL('clicked()'), self.update_selected_cb)
        formlayout.addRow(self.live_update_button)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_joint_angles)
        formlayout.addRow(self.pose_button)
   
        #Controls for getting the current joint states
        self.joint_angs_list = []

        self.list_box = QWidget(pbox)
        self.list_box_layout = QHBoxLayout(self.list_box)
        self.list_box_layout.setMargin(0)

        self.list_widget = QListWidget(self.list_box)
        self.rcommander.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)
        self.list_box_layout.addWidget(self.list_widget)

        self.list_widget_buttons = QWidget(pbox)
        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)

        self.move_up_button = QPushButton(self.list_widget_buttons) #
        self.move_up_button.setText("")
        icon = QIcon()
        base_path = roslib.packages.get_pkg_dir('rcommander_pr2_gui')
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/UpButton.png")), QIcon.Normal, QIcon.Off)
        self.move_up_button.setIcon(icon)
        self.move_up_button.setObjectName("up_button")
        self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
        self.move_up_button.setToolTip('Move Up')

        self.move_down_button = QPushButton(self.list_widget_buttons)
        self.move_down_button.setText("")
        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/DownButton.png")), QIcon.Normal, QIcon.Off)
        self.move_down_button.setIcon(icon)
        self.move_down_button.setObjectName("down_button")
        self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
        self.move_down_button.setToolTip('Move Down')
	
        self.add_joint_set_button = QPushButton(self.list_widget_buttons) #
        self.add_joint_set_button.setText("")
        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/AddButton.png")), QIcon.Normal, QIcon.Off)
        self.add_joint_set_button.setIcon(icon)
        self.add_joint_set_button.setObjectName("add_button")
        self.rcommander.connect(self.add_joint_set_button, SIGNAL('clicked()'), self.add_joint_set_cb)
        #QToolTip.add(add_joint_set_button, 'Add')
        self.add_joint_set_button.setToolTip('Add')

        self.remove_joint_set_button = QPushButton(self.list_widget_buttons)
        self.remove_joint_set_button.setText("")
        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/RemoveButton.png")), QIcon.Normal, QIcon.Off)
        self.remove_joint_set_button.setIcon(icon)
        self.remove_joint_set_button.setObjectName("remove_button")
        self.rcommander.connect(self.remove_joint_set_button, SIGNAL('clicked()'), self.remove_pose_cb)
        self.remove_joint_set_button.setToolTip('Remove')

        self.save_button = QPushButton(self.list_widget_buttons)
        self.save_button.setText("")
        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/SaveButton.png")), QIcon.Normal, QIcon.Off)
        self.save_button.setIcon(icon)
        self.save_button.setObjectName("save_button")
        self.rcommander.connect(self.save_button, SIGNAL('clicked()'), self.save_button_cb)
        self.save_button.setToolTip('Save')
        
        spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding) #

        self.lbb_hlayout.addWidget(self.add_joint_set_button) #
        self.lbb_hlayout.addWidget(self.remove_joint_set_button)
        self.lbb_hlayout.addWidget(self.save_button)
        self.lbb_hlayout.addItem(spacer) #
        self.lbb_hlayout.addWidget(self.move_up_button) #
        self.lbb_hlayout.addWidget(self.move_down_button) #
        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)

        formlayout.addRow('\n', self.list_box)      #
        formlayout.addRow('&Sequence:', self.list_box) #
        formlayout.addRow(self.list_box)
        formlayout.addRow(self.list_widget_buttons)
        self.reset()

    def set_update_mode(self, on):
        if on:
            self.live_update_button.setText('End Live Update')
            self.live_update_button.setEnabled(True)
            self.pose_button.setEnabled(False)
            self.status_bar_timer.start(30)
            self.current_update_color = create_color(0,180,75,255)
        else:
            self.live_update_button.setText('Live Update')
            self.pose_button.setEnabled(True)
            self.status_bar_timer.stop()
            self.current_update_color = create_color(0,0,0,255)

        for name in self.joint_name_fields:      
            palette = self.current_update_color
            exec('self.%s.setPalette(palette)' % name)

        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        for idx, name in enumerate(self.joint_name_fields):
            exec('line_edit = self.%s' % name)
            self._check_limit(arm, line_edit.value(), name)
            #line_edit.setValue(line_edit.value())

    def update_selected_cb(self):
        if self.live_update_button.text() == 'Live Update':
            self.set_update_mode(True)
        else:
            self.set_update_mode(False)

    def _refill_list_widget(self, joints_list):
        self.list_widget.clear()
        for d in joints_list:
            self.list_widget.addItem(d['name'])

    def get_current_joint_angles(self):
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        if ('left' == arm):
            arm_obj = self.rcommander.robot.left
        else:
            arm_obj = self.rcommander.robot.right

        pose_mat = arm_obj.pose()

        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(pose_mat[idx, 0])
            exec('line_edit = self.%s' % name)
            line_edit.setValue(deg)


    def _has_name(self, test_name):
        for rec in self.joint_angs_list:
            if rec['name'] == test_name:
                return True
            else:
                return False

    def _create_name(self):
        idx = len(self.joint_angs_list)
        tentative_name = 'point%d' % idx 

        while self._has_name(tentative_name):
            idx = idx + 1
            tentative_name = 'point%d' % idx 

        return tentative_name

    def item_selection_changed_cb(self):
        self.set_update_mode(False)
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return
        idx = self._find_index_of(str(selected[0].text()))
        self.curr_selected = idx

        joint_angs = self.joint_angs_list[idx]['angs']
        self._set_joints_to_fields(joint_angs)
        self.time_box.setValue(self.joint_angs_list[idx]['time'])
        self._time_changed_validate(self.joint_angs_list[idx]['time'])

        #self.status_bar_timer.stop()
        #self.pose_button.setEnabled(True)

    def add_joint_set_cb(self):
        #Create a new string, check to see whether it's in the current list
        name = self._create_name()
        #self.list_widget.addItem(name)
        self.joint_angs_list.append({'name':name, 
            'time': self.time_box.value(), 
            'angs': self._read_joints_from_fields(True)})
        self._refill_list_widget(self.joint_angs_list)

    def _find_index_of(self, name):
        for idx, tup in enumerate(self.joint_angs_list):
            if tup['name'] == name:
                return idx
        return None

    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.joint_angs_list.pop(idx)
        self.joint_angs_list.insert(idx-1, item)

        #refresh
        self._refill_list_widget(self.joint_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))

    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.joint_angs_list.pop(idx)
        self.joint_angs_list.insert(idx+1, item)

        #refresh
        self._refill_list_widget(self.joint_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))

    def _selected_idx(self):
        #Get currently selected
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return None
        sname = str(selected[0].text())

        #Remove it from list_widget and joint_angs_list
        idx = self._find_index_of(sname)
        return idx

    def save_button_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        el = self.joint_angs_list[idx]
        self.joint_angs_list[idx] = {'name': el['name'],
                                     'time': self.time_box.value(), 
                                     'angs': self._read_joints_from_fields(True)}

    def remove_pose_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        #self.list_widget.takeItem(idx)
        #self.list_widget.removeItemWidget(selected[0])
        if idx == None:
            raise RuntimeError('Inconsistency detected in list')
        else:
            self.joint_angs_list.pop(idx)
        self._refill_list_widget(self.joint_angs_list)

    def _read_joints_from_fields(self, limit_ranges=False):
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        if arm == 'left':
            idx = 0
            pref = 'l_'
        else:
            idx = 1
            pref = 'r_'

        limits = self.limits[idx]
        joints = []
        for name in self.joint_name_fields:
            #exec('rad = np.radians(float(str(self.%s.text())))' % name)
            exec('rad = np.radians(self.%s.value())' % name)
            if limit_ranges and limits.has_key(pref+name):
                mn, mx = limits[pref+name]
                nrad = max(min(rad, mx-.005), mn+.005)
                #if rad != nrad:
                #    print pref+name, nrad, rad
                rad = nrad
            joints.append(rad)
        return joints

    def _set_joints_to_fields(self, joints):
        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(joints[idx])
            exec('line_edit = self.%s' % name)
            line_edit.setValue(deg)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        if (self.joint_angs_list == None or len(self.joint_angs_list) == 0) and (name != None):
            return None
    
        #sstate = JointSequenceState(nname, str(self.arm_box.currentText()), self._read_joints_from_fields())

        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        sstate = JointSequenceState(nname, arm, self.joint_angs_list)
        #sstate.set_robot(self.rcommander.robot)
        return sstate

    def set_node_properties(self, my_node):
        self.joint_angs_list = my_node.joint_waypoints
        self._refill_list_widget(self.joint_angs_list)
        #self.arm_box.setCurrentIndex(self.arm_box.findText(my_node.arm))
        if 'left' == my_node.arm:
            self.arm_radio_buttons[0].setChecked(True)
        if my_node.arm == 'right':
            self.arm_radio_buttons[1].setChecked(True)
        self.list_widget.setCurrentItem(self.list_widget.item(0))

    def reset(self):
        self.arm_radio_buttons[0].setChecked(True)
        #self.arm_box.setCurrentIndex(self.arm_box.findText('left'))
        for name in self.joint_name_fields:
            exec('self.%s.setValue(0)' % name)

        #self.update_checkbox.setCheckState(False)
        self.status_bar_timer.stop()
        self.pose_button.setEnabled(True)

class JointSequenceState(tu.StateBase): 

    def __init__(self, name, arm, joint_waypoints):
        tu.StateBase.__init__(self, name)
        self.arm = arm
        self.joint_waypoints = joint_waypoints

    def get_smach_state(self):
        return JointSequenceStateSmach(self.arm, self.joint_waypoints)

class JointSequenceStateSmach(smach.State): 

    TIME_OUT_FACTOR = 3.

    def __init__(self, arm, joint_waypoints):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed', 'aborted'], 
                             input_keys = [], output_keys = [])
        self.arm = arm
        self.joint_waypoints = joint_waypoints
        self.arm_obj = None

    def set_robot(self, pr2):
        if pr2 == None:
            return

        if self.arm == 'left':
            self.arm_obj = pr2.left

        if self.arm == 'right':
            self.arm_obj = pr2.right
        self.controller_manager = pr2.controller_manager

    def execute(self, userdata):
        status, started, stopped = self.controller_manager.joint_mode(self.arm)

        #Construct trajectory command
        times = []
        wps = []
        for d in self.joint_waypoints:
            wps.append(np.matrix(d['angs']).T)
            times.append(d['time'])

        #print 'move_tool: sending poses'
        #print 'MOVE_TOOL: wps', wps
        #print 'MOVE_TOOL: times', times

        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
        client = self.arm_obj.client
        state = client.get_state()

        #Monitor execution
        trajectory_time_out = JointSequenceStateSmach.TIME_OUT_FACTOR * np.sum(times)
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('JointSequenceState: preempt requested')
                client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > trajectory_time_out:
                client.cancel_goal()
                rospy.loginfo('JointSequenceState: timed out!')
                succeeded = False
                break

            #print tu.goal_status_to_string(state)
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('JointSequenceState: Succeeded!')
                    succeeded = True
                break

            state = client.get_state()

            r.sleep()

        #print 'end STATE', state

        self.controller_manager.switch(stopped, started)

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        if state == am.GoalStatus.ABORTED:
            return 'aborted'

        return 'failed'

#    def __init_unpicklables__(self):
#
#class JointSequenceState(smach.State, tu.StateBase): 
#
#    TIME_OUT_FACTOR = 3.
#
#    def __init__(self, name, arm, joint_waypoints):
#        self.name = name
#        tu.StateBase.__init__(self, self.name)
#        self.arm = arm
#        self.joint_waypoints = joint_waypoints
#        self.arm_obj = None
#        self.__init_unpicklables__()
#
#    def execute(self, userdata):
#        self.controller_manager.joint_mode(self.arm)
#
#        #Construct trajectory command
#        times = []
#        wps = []
#        for d in self.joint_waypoints:
#            wps.append(np.matrix(d['angs']).T)
#            times.append(d['time'])
#
#        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
#        client = self.arm_obj.client
#        state = client.get_state()
#
#        #Monitor execution
#        trajectory_time_out = JointSequenceState.TIME_OUT_FACTOR * np.sum(times)
#        succeeded = False
#        preempted = False
#        r = rospy.Rate(30)
#        start_time = rospy.get_time()
#        while True:
#            #we have been preempted
#            if self.preempt_requested():
#                rospy.loginfo('JointSequenceState: preempt requested')
#                client.cancel_goal()
#                self.service_preempt()
#                preempted = True
#                break
#
#            if (rospy.get_time() - start_time) > trajectory_time_out:
#                client.cancel_goal()
#                rospy.loginfo('JointSequenceState: timed out!')
#                succeeded = False
#                break
#
#            #print tu.goal_status_to_string(state)
#            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
#                if state == am.GoalStatus.SUCCEEDED:
#                    rospy.loginfo('JointSequenceState: Succeeded!')
#                    succeeded = True
#                break
#
#            state = client.get_state()
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
#
#    def set_robot(self, pr2):
#        if self.arm == 'left':
#            self.arm_obj = pr2.left
#
#        if self.arm == 'right':
#            self.arm_obj = pr2.right
#
#    def __init_unpicklables__(self):
#        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = [], output_keys = [])
#        self.controller_manager = ControllerManager()
#
#    def __getstate__(self):
#        state = tu.StateBase.__getstate__(self)
#        my_state = [self.name, self.arm, self.joint_waypoints] 
#        return {'state_base': state, 'self': my_state}
#
#    def __setstate__(self, state):
#        tu.StateBase.__setstate__(self, state['state_base'])
#        self.name, self.arm, self.joint_waypoints = state['self']
#        self.__init_unpicklables__()


