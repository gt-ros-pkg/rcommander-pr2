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
import pr2_utils as p2u


class JointSequenceTool(tu.ToolBase, p2u.JointTool):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'joint_sequence', 'Joint Sequence', JointSequenceState)
        p2u.JointTool.__init__(self, rcommander.robot, rcommander)
        self.min_time_service = rospy.ServiceProxy('min_time_to_move', MinTime, persistent=True)
        #self.current_selected = None
        self.reset_live_update = True
        self.element_will_be_added = False

    def _value_changed_validate(self, value, joint):
        arm = self.get_arm_radio()
        self.check_joint_limits(arm, value, joint)
        self._check_time_validity(self.time_box.value())

    def get_joint_angs_list(self):
        return self.list_manager.get_data(clean=True)

    def _check_time_validity(self, value):
        r,g,b = 0,0,0
        palette = QPalette(QColor(r, g, b, 255))
        palette.setColor(QPalette.Text, QColor(r, g, b, 255))
        self.time_box.setPalette(palette)

        #arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        arm = self.get_arm_radio()
        if arm == 'left':
            #idx = 0
            pref = 'l_'
            left_arm = True
        else:
            #idx = 1
            pref = 'r_'
            left_arm = False

        if len(self.get_joint_angs_list()) == 0:
            return

        if self.list_manager.get_selected_idx() == None:
            ref = self.get_joint_angs_list()[-1]
        else:
            pidx = self.list_manager.get_selected_idx() - 1
            if pidx < 0:
                return 
            else:
                ref = self.get_joint_angs_list()[pidx]

        start_point = tm.JointTrajectoryPoint()
        start_point.velocities = [0.] * len(p2u.JOINT_NAME_FIELDS)
        for name in p2u.JOINT_NAME_FIELDS:
            exec('box = self.%s' % name)
            start_point.positions.append(np.radians(box.value()))

        end_point = tm.JointTrajectoryPoint()
        end_point.velocities = [0.] * len(p2u.JOINT_NAME_FIELDS)
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
                return

    def _time_changed_validate(self, value):
        self._check_time_validity(value)

    def get_current_data_cb(self):
        return {'time': self.time_box.value(), 
                'angs': self.read_joints_from_fields(True)}

    def set_current_data_cb(self, data):
        if self.reset_live_update and not self.element_will_be_added:
            # - This is here so that when user selects another point live
            #   update is turned OFF but add also uses this function, which turns
            #   OFF live update!
            # - The toggle button function only remembers the element activated
            #   when it was clicked.
            self.set_update_mode(False) 

        self.set_joints_to_fields(data['angs'])
        self.time_box.setValue(data['time'])
        self._time_changed_validate(data['time'])

        if self.element_will_be_added:
            self.element_will_be_added = False

    def live_update_cb(self):
        pose_mat = self.get_robot_joint_angles()
        self.reset_live_update = False
        self.list_manager.display_record({'time': self.time_box.value(),
                                          'angs': pose_mat.A1.tolist()})
        self.reset_live_update = True

    def live_update_toggle_cb(self, state):
        if not state:
            self.reset_live_update = False
            self.list_manager.set_selected_by_name(self.list_manager.get_selected_name())
            self.reset_live_update = True

    def add_element_cb(self):
        self.element_will_be_added = True

    #    #Live update is off at this point, and we have a new point
    #    #If current exists, overwrite its old value, reset variable current
    #    #   (do a _refill to make sure ordering is ok)
    #    #   (reselect element)

    #    #turn live update back on, this will save the just added element!

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        items_to_monitor = []

        fields, arm_radio_boxes, buttons = self.make_joint_boxes(pbox, self.rcommander)
        formlayout.addRow('&Arm', arm_radio_boxes)
        for field in fields:
            formlayout.addRow(field['name'], field['item'])
            vchanged_func = ft.partial(self._value_changed_validate, joint=field['joint'])
            self.rcommander.connect(field['item'], SIGNAL('valueChanged(double)'), vchanged_func)
            items_to_monitor.append(field['item'])

        self.time_box = tu.double_spin_box(pbox, 0, 1000, .2)
        self.time_box.setValue(1.)
        formlayout.addRow('&Time', self.time_box)
        self.rcommander.connect(self.time_box, SIGNAL('valueChanged(double)'), self._time_changed_validate)
        items_to_monitor.append(self.time_box)

        for button in buttons:
            formlayout.addRow(button)

        #Controls for getting the current joint states
        self.list_manager = p2u.ListManager(self.get_current_data_cb, self.set_current_data_cb, 
                                            self.add_element_cb, name_preffix='point')
        list_widgets = self.list_manager.make_widgets(pbox, self.rcommander)
        self.list_manager.monitor_changing_values_in(self.rcommander, items_to_monitor)

        formlayout.addRow('\n', list_widgets[0])
        formlayout.addRow('&Sequence:', list_widgets[0])
        for gb in list_widgets:
            formlayout.addRow(gb)
        self.reset()

    def new_node(self, name=None):
        self.list_manager.save_currently_selected_item()
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        #if (self.joint_angs_list == None or len(self.joint_angs_list) == 0) and (name != None):
        if (len(self.get_joint_angs_list()) == 0) and (name != None):
            return None
        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        sstate = JointSequenceState(nname, arm, self.list_manager.get_data())
        return sstate

    def set_node_properties(self, my_node):
        self.list_manager.set_data(my_node.joint_waypoints)
        self.list_manager.select_default_item()
        self.set_arm_radio(my_node.arm)

    def reset(self):
        self.set_arm_radio('left')
        self.set_all_fields_to_zero()
        self.pose_button.setEnabled(True)
        self.list_manager.reset()
        self.stop_timer()


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
            wps.append(np.matrix(d['data']['angs']).T)
            times.append(d['data']['time'])

        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
        client = self.arm_obj.client
        state = client.get_state()

        #Monitor execution
        trajectory_time_out = JointSequenceStateSmach.TIME_OUT_FACTOR * np.sum(times)
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
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

        #formlayout.addRow('&Sequence:', self.list_box) #
        #formlayout.addRow(self.list_box)
        #formlayout.addRow(self.list_widget_buttons)

        #self.list_box = QWidget(pbox)
        #self.list_box_layout = QHBoxLayout(self.list_box)
        #self.list_box_layout.setMargin(0)

        #self.list_widget = QListWidget(self.list_box)
        #self.rcommander.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)
        #self.list_box_layout.addWidget(self.list_widget)

        #self.list_widget_buttons = QWidget(pbox)
        #self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)

        #self.move_up_button = QPushButton(self.list_widget_buttons) #
        #self.move_up_button.setText("")
        #icon = QIcon()
        #base_path = roslib.packages.get_pkg_dir('rcommander_pr2_gui')
        #icon.addPixmap(QPixmap(pt.join(base_path, "icons/UpButton.png")), QIcon.Normal, QIcon.Off)
        #self.move_up_button.setIcon(icon)
        #self.move_up_button.setObjectName("up_button")
        #self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
        #self.move_up_button.setToolTip('Move Up')

        #self.move_down_button = QPushButton(self.list_widget_buttons)
        #self.move_down_button.setText("")
        #icon = QIcon()
        #icon.addPixmap(QPixmap(pt.join(base_path, "icons/DownButton.png")), QIcon.Normal, QIcon.Off)
        #self.move_down_button.setIcon(icon)
        #self.move_down_button.setObjectName("down_button")
        #self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
        #self.move_down_button.setToolTip('Move Down')
	
        #self.add_joint_set_button = QPushButton(self.list_widget_buttons) #
        #self.add_joint_set_button.setText("")
        #icon = QIcon()
        #icon.addPixmap(QPixmap(pt.join(base_path, "icons/AddButton.png")), QIcon.Normal, QIcon.Off)
        #self.add_joint_set_button.setIcon(icon)
        #self.add_joint_set_button.setObjectName("add_button")
        #self.rcommander.connect(self.add_joint_set_button, SIGNAL('clicked()'), self.add_joint_set_cb)
        ##QToolTip.add(add_joint_set_button, 'Add')
        #self.add_joint_set_button.setToolTip('Add')

        #self.remove_joint_set_button = QPushButton(self.list_widget_buttons)
        #self.remove_joint_set_button.setText("")
        #icon = QIcon()
        #icon.addPixmap(QPixmap(pt.join(base_path, "icons/RemoveButton.png")), QIcon.Normal, QIcon.Off)
        #self.remove_joint_set_button.setIcon(icon)
        #self.remove_joint_set_button.setObjectName("remove_button")
        #self.rcommander.connect(self.remove_joint_set_button, SIGNAL('clicked()'), self.remove_pose_cb)
        #self.remove_joint_set_button.setToolTip('Remove')

        #self.save_button = QPushButton(self.list_widget_buttons)
        #self.save_button.setText("")
        #icon = QIcon()
        #icon.addPixmap(QPixmap(pt.join(base_path, "icons/SaveButton.png")), QIcon.Normal, QIcon.Off)
        #self.save_button.setIcon(icon)
        #self.save_button.setObjectName("save_button")
        #self.rcommander.connect(self.save_button, SIGNAL('clicked()'), self.save_button_cb)
        #self.save_button.setToolTip('Save')
        
        #spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding) #

        #self.lbb_hlayout.addWidget(self.add_joint_set_button) #
        #self.lbb_hlayout.addWidget(self.remove_joint_set_button)
        #self.lbb_hlayout.addWidget(self.save_button)
        #self.lbb_hlayout.addItem(spacer) #
        #self.lbb_hlayout.addWidget(self.move_up_button) #
        #self.lbb_hlayout.addWidget(self.move_down_button) #
        #self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)



    #def _has_name(self, test_name):
    #    for rec in self.joint_angs_list:
    #        if rec['name'] == test_name:
    #            return True
    #        else:
    #            return False

    #def _create_name(self):
    #    idx = len(self.joint_angs_list)
    #    tentative_name = 'point%d' % idx 

    #    while self._has_name(tentative_name):
    #        idx = idx + 1
    #        tentative_name = 'point%d' % idx 

    #    return tentative_name

    #def item_selection_changed_cb(self):
    #    self.set_update_mode(False)
    #    selected = self.list_widget.selectedItems()
    #    if len(selected) == 0:
    #        return
    #    idx = self._find_index_of(str(selected[0].text()))
    #    self.curr_selected = idx

    #    joint_angs = self.joint_angs_list[idx]['angs']
    #    self._set_joints_to_fields(joint_angs)
    #    self.time_box.setValue(self.joint_angs_list[idx]['time'])
    #    self._time_changed_validate(self.joint_angs_list[idx]['time'])

    #    #self.status_bar_timer.stop()
    #    #self.pose_button.setEnabled(True)

    #def add_joint_set_cb(self):
    #    #Create a new string, check to see whether it's in the current list
    #    name = self._create_name()
    #    #self.list_widget.addItem(name)
    #    self.joint_angs_list.append({'name':name, 
    #        'time': self.time_box.value(), 
    #        'angs': self._read_joints_from_fields(True)})
    #    self._refill_list_widget(self.joint_angs_list)

    #def _find_index_of(self, name):
    #    for idx, tup in enumerate(self.joint_angs_list):
    #        if tup['name'] == name:
    #            return idx
    #    return None

    #def move_up_cb(self):
    #    #get the current index
    #    idx = self._selected_idx()
    #    if idx == None:
    #        return

    #    #pop & insert it
    #    item = self.joint_angs_list.pop(idx)
    #    self.joint_angs_list.insert(idx-1, item)

    #    #refresh
    #    self._refill_list_widget(self.joint_angs_list)
    #    self.list_widget.setCurrentItem(self.list_widget.item(idx-1))

    #def move_down_cb(self):
    #    #get the current index
    #    idx = self._selected_idx()
    #    if idx == None:
    #        return

    #    #pop & insert it
    #    item = self.joint_angs_list.pop(idx)
    #    self.joint_angs_list.insert(idx+1, item)

    #    #refresh
    #    self._refill_list_widget(self.joint_angs_list)
    #    self.list_widget.setCurrentItem(self.list_widget.item(idx+1))

    #def _selected_idx(self):
    #    #Get currently selected
    #    selected = self.list_widget.selectedItems()
    #    if len(selected) == 0:
    #        return None
    #    sname = str(selected[0].text())

    #    #Remove it from list_widget and joint_angs_list
    #    idx = self._find_index_of(sname)
    #    return idx

    #def save_button_cb(self):
    #    idx = self._selected_idx()
    #    if idx == None:
    #        return
    #    el = self.joint_angs_list[idx]
    #    self.joint_angs_list[idx] = {'name': el['name'],
    #                                 'time': self.time_box.value(), 
    #                                 'angs': self._read_joints_from_fields(True)}

    #def remove_pose_cb(self):
    #    idx = self._selected_idx()
    #    if idx == None:
    #        return
    #    #self.list_widget.takeItem(idx)
    #    #self.list_widget.removeItemWidget(selected[0])
    #    if idx == None:
    #        raise RuntimeError('Inconsistency detected in list')
    #    else:
    #        self.joint_angs_list.pop(idx)
    #    self._refill_list_widget(self.joint_angs_list)
