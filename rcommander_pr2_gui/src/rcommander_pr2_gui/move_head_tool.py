import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import actionlib_msgs.msg as am
import numpy as np
import smach
import roslib
import os.path as pt
import pypr2.pr2_utils as p2u




class MoveHeadTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_head_ang', 'Move Head (ang)', MoveHeadState)
        self.head_angs_list = None
        self.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.status_bar_timer = QTimer()
        self.rcommander.connect(self.status_bar_timer, SIGNAL('timeout()'), self.live_update_cb)
        self.live_update = False
        self.current_update_color = p2u.create_color(0,0,0,255)

    def live_update_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.list_manager.display_record({'angs': poses.A1.tolist(), 'time': self.time_box.value()})

    def current_pose_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.joint_boxes[0].setValue(np.degrees(poses[0,0]))
        self.joint_boxes[1].setValue(np.degrees(poses[1,0]))

    def update_selected_cb(self):
        if self.live_update_button.text() == 'Live Update':
            self.set_update_mode(True)
        else:
            self.set_update_mode(False)

    def get_current_data_cb(self):
        head_angs = []
        for name in self.joint_names:
            exec('obj = self.%s' % name)
            head_angs.append(np.radians(obj.value()))
        return {'angs': head_angs,
                'time': self.time_box.value()}

    def set_current_data_cb(self, data):
        for idx, name in enumerate(self.joint_names):
            exec('obj = self.%s' % name)
            obj.setValue(np.degrees(data['angs'][idx]))
        self.time_box.setValue(data['time'])

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
    
        items_to_monitor = []
        self.joint_boxes = []
        for jname in self.joint_names:
            spin_box = tu.double_spin_box(pbox, -360, 360, 1.)
            exec("self.%s = spin_box" % jname)
            self.joint_boxes.append(spin_box)
            items_to_monitor.append(spin_box)
            formlayout.addRow('&%s' % jname, spin_box)

        self.time_box = tu.double_spin_box(pbox, 0, 1000, .5)
        items_to_monitor.append(self.time_box)
        formlayout.addRow('&Time', self.time_box)

        self.live_update_button = QPushButton(pbox)
        self.live_update_button.setText('Live Update')
        self.rcommander.connect(self.live_update_button, SIGNAL('clicked()'), self.update_selected_cb)
        formlayout.addRow(self.live_update_button)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Set to Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.current_pose_cb)
        formlayout.addRow(self.pose_button)

        self.list_manager = p2u.ListManager(self.get_current_data_cb, self.set_current_data_cb, None, name_preffix='point')
        list_widgets = self.list_manager.make_widgets(pbox, self.rcommander)
        self.list_manager.monitor_changing_values_in(self.rcommander, items_to_monitor)
        for gb in list_widgets:
            formlayout.addRow(gb)
        self.reset()

    def new_node(self, name=None):
        self.list_manager.save_currently_selected_item()
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        data = self.list_manager.get_data()
        if len(data) == 0 and name != None:
            return None
        return MoveHeadState(nname, data)

    def reset(self):
        for box in self.joint_boxes:
            box.setValue(0)
        self.time_box.setValue(1.)
        self.list_manager.reset()
        self.list_manager.set_default_selection()

	    #self.update_checkbox.setCheckState(False)
        #self.status_bar_timer.stop()
        #self.current_pose_button.setEnabled(True)

    def set_node_properties(self, my_node):
        self.list_manager.set_data(my_node.joint_waypoints)
        self.list_manager.select_default_item()

        #self.joint_boxes[0].setValue(np.degrees(my_node.poses[0,0]))
        #self.joint_boxes[1].setValue(np.degrees(my_node.poses[1,0]))
        ##self.time_box.setValue(my_node.mot_time) 
        #self.head_angs_list = my_node.joint_waypoints
        #self._refill_list_widget(my_node.joint_waypoints)
        #self.list_widget.setCurrentItem(self.list_widget.item(0))

    #def update_selected_cb(self, state):
    #    # checked
    #    if state == 2:
    #        self.status_bar_timer.start(30)
    #        self.current_pose_button.setEnabled(False)
	#    self.get_current_joint_angles_cb()
    #    # unchecked
    #    if state == 0:
    #        self.status_bar_timer.stop()
    #        self.current_pose_button.setEnabled(True)

    def set_update_mode(self, on):
        self.live_update = on

        if on:
            self.live_update_toggle_cb(self.live_update)

        if self.live_update:
            self.live_update_button.setText('End Live Update')
            self.live_update_button.setEnabled(True)
            self.pose_button.setEnabled(False)
            self.status_bar_timer.start(30)
            self.current_update_color = p2u.create_color(0,180,75,255)
        else:
            self.live_update_button.setText('Live Update')
            self.pose_button.setEnabled(True)
            self.status_bar_timer.stop()
            self.current_update_color = p2u.create_color(0,0,0,255)

        for name in self.joint_names:      
            palette = self.current_update_color
            exec('self.%s.setPalette(palette)' % name)

        #arm = self.get_arm_radio()
        #for idx, name in enumerate(JOINT_NAME_FIELDS):
        #    exec('line_edit = self.%s' % name)
        #    self.check_joint_limits(arm, line_edit.value(), name)

        if not on:
            self.live_update_toggle_cb(self.live_update)

    def live_update_toggle_cb(self, state):
        if not state:
            self.reset_live_update = False
            selected_name = self.list_manager.get_selected_name()
            if selected_name != None:
                self.list_manager.set_selected_by_name(selected_name)
            self.reset_live_update = True

class MoveHeadState(tu.StateBase):

    def __init__(self, name, joint_waypoints):
        tu.StateBase.__init__(self, name)
        self.joint_waypoints = joint_waypoints

    def get_smach_state(self):
        return MoveHeadStateSmach(self.joint_waypoints)


class MoveHeadStateSmach(smach.State):

    TIME_OUT_FACTOR = 3.

    def __init__(self, joint_waypoints):
        smach.State.__init__(self, outcomes=['preempted', 'done'], 
			     input_keys=[], output_keys=[])
        self.joint_waypoints = joint_waypoints

    def set_robot(self, robot):
        if robot != None:
            self.head_obj = robot.head
            self.controller_manager = robot.controller_manager

    def execute(self, userdata):
        #status, started, stopped = self.controller_manager.joint_mode(self.head_obj)

        #Construct trajectory command
        times = []
        wps = []
        for x in self.joint_waypoints:
            wps.append(np.matrix(x['data']['angs']).T)
            times.append(x['data']['time'])

        self.head_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)))
        #client = self.head_obj.client
        #state = client.get_state()

        #Monitor execution
        trajectory_time_out = MoveHeadStateSmach.TIME_OUT_FACTOR * np.sum(times)
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < trajectory_time_out:
            r.sleep()
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        return 'done'




#class MoveHeadTool(tu.ToolBase):
#
#    def __init__(self, rcommander):
#        tu.ToolBase.__init__(self, rcommander, 'move_head_ang', 'Move Head (ang)', MoveHeadState)
#        self.head_angs_list = None
#        self.joint_names = ["head_pan_joint", "head_tilt_joint"]
#        self.status_bar_timer = QTimer()
#        self.rcommander.connect(self.status_bar_timer, SIGNAL('timeout()'), self.get_current_joint_angles_cb)
#
#    def fill_property_box(self, pbox):
#        formlayout = pbox.layout()
#	
#        joint_names = self.rcommander.robot.head.get_joint_names()
#        self.joint_boxes = []
#        for jname in joint_names:
#            #joint_box = QDoubleSpinBox(pbox)
#            exec("self.%s = QDoubleSpinBox(pbox)" % jname)
#            exec('box = self.%s' % jname)
#            box.setMinimum(-9999.)
#            box.setMaximum(9999.)
#            box.setSingleStep(1.)
#            self.joint_boxes.append(box)
#            formlayout.addRow('&%s' % jname, box)
#
#        self.head_angs_list = []
#        self.time_box = QDoubleSpinBox(pbox)
#        self.time_box.setMinimum(0)
#        self.time_box.setMaximum(1000.)
#        self.time_box.setSingleStep(.5)
#        formlayout.addRow('&Time', self.time_box)
#
#        self.update_checkbox = QCheckBox(pbox) 
#        self.update_checkbox.setTristate(False)
#        formlayout.addRow('&Live Update', self.update_checkbox)
#        self.rcommander.connect(self.update_checkbox, SIGNAL('stateChanged(int)'), self.update_selected_cb)
#
#        self.current_pose_button = QPushButton(pbox)
#        self.current_pose_button.setText('Current Pose')
#        self.rcommander.connect(self.current_pose_button, 
#                SIGNAL('clicked()'), self.get_current_joint_angles_cb)
#        formlayout.addRow(self.current_pose_button)
#     
##widet box 
#        self.list_box = QWidget(pbox)
#        self.list_box_layout = QHBoxLayout(self.list_box)
#        self.list_box_layout.setMargin(0)
#
#        self.list_widget = QListWidget(self.list_box)
#        self.rcommander.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)
#        self.list_box_layout.addWidget(self.list_widget)
#
#        #self.movement_buttons_widget = QWidget(self.list_box)
#        #self.movement_buttons_widgetl = QVBoxLayout(self.movement_buttons_widget)
#        #self.movement_buttons_widgetl.setMargin(0)
#
#        self.list_widget_buttons = QWidget(pbox)
#        self.list_widget_buttons2 = QWidget(pbox)
#        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)
#	#self.lbb_hlayout2 = QHBoxLayout(self.list_widget_buttons2)
#
##UP
#
#        #self. = QPushButton(self.list_widget_buttons)
#        #self.move_up_button.setText('Up')
#        #self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
#        #self.lbb_hlayout.addWidget(self.move_up_button)
##DOWN
#        #self.move_down_button = QPushButton(self.list_widget_buttons)
#        #self.move_down_button.setText('Down')
#        #self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
#        #self.lbb_hlayout.addWidget(self.move_down_button)
#	#self.lbb_hlayout.addSpacing(50)
#
#        #spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
#        #self.lbb_hlayout.addItem(spacer)
#
#	    #formlayout.addRow('&Create a Joint Sequence', self.time_box)
#
#        self.move_up_button = QPushButton(self.list_widget_buttons)
#        self.move_up_button.setText("")
#        icon = QIcon()
#        base_path = roslib.packages.get_pkg_dir('rcommander_pr2_gui')
#        icon.addPixmap(QPixmap(pt.join(base_path, "icons/UpButton.png")), QIcon.Normal, QIcon.Off)
#        self.move_up_button.setIcon(icon)
#        self.move_up_button.setObjectName("up_button")
#        #self. = QPushButton(self.list_widget_buttons)
#        #self.move_up_button.setText('Up')
#        self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
#        #self.lbb_hlayout.addWidget(self.move_up_button)
#        self.move_up_button.setToolTip('Move Up')
#
#        self.move_down_button = QPushButton(self.list_widget_buttons)
#        self.move_down_button.setText("")
#        icon = QIcon()
#        icon.addPixmap(QPixmap(pt.join(base_path,"icons/DownButton.png")), QIcon.Normal, QIcon.Off)
#        self.move_down_button.setIcon(icon)
#        self.move_down_button.setObjectName("down_button")
#        #self.move_down_button = QPushButton(self.list_widget_buttons)
#        #self.move_down_button.setText('Down')
#        self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
#        #self.lbb_hlayout.addWidget(self.move_down_button)
#        #self.lbb_hlayout.addSpacing(50)
#        self.move_down_button.setToolTip('Move Down')
#
#        self.add_head_set_button = QPushButton(self.list_widget_buttons)
#        self.add_head_set_button.setText("")
#        icon = QIcon()
#        icon.addPixmap(QPixmap(pt.join(base_path, "icons/AddButton.png")), QIcon.Normal, QIcon.Off)
#        self.add_head_set_button.setIcon(icon)
#        self.add_head_set_button.setObjectName("add_button")
#        #self.add_head_set_button = QPushButton(self.list_widget_buttons2)
#        #self.add_head_set_button.setText('Add Head Position')
#        self.rcommander.connect(self.add_head_set_button, SIGNAL('clicked()'), self.add_head_set_cb)
#        self.add_head_set_button.setToolTip('Add')
#
#        self.remove_head_set_button = QPushButton(self.list_widget_buttons)
#        self.remove_head_set_button.setText("")
#        icon = QIcon()
#        icon.addPixmap(QPixmap(pt.join(base_path, "icons/RemoveButton.png")), QIcon.Normal, QIcon.Off)
#        self.remove_head_set_button.setIcon(icon)
#        self.remove_head_set_button.setObjectName("remove_button")
#        #self.remove_head_set_button = QPushButton(self.list_widget_buttons2)
#        #self.remove_head_set_button.setText('Remove Head Position')
#        self.rcommander.connect(self.remove_head_set_button, SIGNAL('clicked()'), self.remove_pose_cb)
#        self.remove_head_set_button.setToolTip('Remove')
#
#        self.save_button = QPushButton(self.list_widget_buttons)
#        self.save_button.setText("")
#        icon = QIcon()
#        icon.addPixmap(QPixmap(pt.join(base_path, "icons/SaveButton.png")), QIcon.Normal, QIcon.Off)
#        self.save_button.setIcon(icon)
#        self.save_button.setObjectName("save_button")
#        #self.save_button = QPushButton(self.list_widget_buttons2)
#        #self.save_button.setText('Save Sequence')
#        self.rcommander.connect(self.save_button, SIGNAL('clicked()'), self.save_button_cb)
#        self.save_button.setToolTip('Save')
#        
#        spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
#        self.lbb_hlayout.addWidget(self.add_head_set_button)
#        self.lbb_hlayout.addWidget(self.remove_head_set_button)
#        self.lbb_hlayout.addWidget(self.save_button)
#        self.lbb_hlayout.addItem(spacer)
#        self.lbb_hlayout.addWidget(self.move_up_button)
#        self.lbb_hlayout.addWidget(self.move_down_button)
#        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)
#
#        formlayout.addRow('\n', self.list_box)      
#        formlayout.addRow('&Sequence:', self.list_box)
#        formlayout.addRow(self.list_box)
#        formlayout.addRow(self.list_widget_buttons)
#        #formlayout.addRow(self.list_widget_buttons2)
#        self.reset()
#
#    def _refill_list_widget(self, joints_list):
#        self.list_widget.clear()
#        for d in joints_list:
#            self.list_widget.addItem(d['name'])
#
#    def get_current_joint_angles_cb(self):
#        poses = self.rcommander.robot.head.pose()
#        self.joint_boxes[0].setValue(np.degrees(poses[0,0]))
#        self.joint_boxes[1].setValue(np.degrees(poses[1,0]))
#
#    def set_node_properties(self, my_node):
#        self.joint_boxes[0].setValue(np.degrees(my_node.poses[0,0]))
#        self.joint_boxes[1].setValue(np.degrees(my_node.poses[1,0]))
#        #self.time_box.setValue(my_node.mot_time) 
#        self.head_angs_list = my_node.joint_waypoints
#        self._refill_list_widget(my_node.joint_waypoints)
#        self.list_widget.setCurrentItem(self.list_widget.item(0))
#
##added   
#    def item_selection_changed_cb(self):
#        selected = self.list_widget.selectedItems()
#        if len(selected) == 0:
#            return
#        idx = self._find_index_of(str(selected[0].text()))
#        head_angs = self.head_angs_list[idx]['angs']
#        self._set_joints_to_fields(head_angs)
#        self.time_box.setValue(self.head_angs_list[idx]['time'])
##added
#    def _set_joints_to_fields(self, joints):
#        for idx, name in enumerate(self.joint_names):
#            deg = np.degrees(joints[idx])
#            exec('line_edit = self.%s' % name)
#            line_edit.setValue(deg)
#
##added
#    def _has_name(self, test_name):
#        for rec in self.head_angs_list:
#            if rec['name'] == test_name:
#                return True
#            else:
#                return False
#
##added
#    def move_up_cb(self):
#        #get the current index
#        idx = self._selected_idx()
#        if idx == None:
#            return
#
#        #pop & insert it
#        item = self.head_angs_list.pop(idx)
#        self.head_angs_list.insert(idx-1, item)
#
#        #refresh
#        self._refill_list_widget(self.head_angs_list)
#        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))
#
##added
#    def move_down_cb(self):
#        #get the current index
#        idx = self._selected_idx()
#        if idx == None:
#            return
#
#        #pop & insert it
#        item = self.head_angs_list.pop(idx)
#        self.head_angs_list.insert(idx+1, item)
#
#        #refresh
#        self._refill_list_widget(self.head_angs_list)
#        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))
#
##added
#    def add_head_set_cb(self):
#        #Create a new string, check to see whether it's in the current list
#        name = self._create_name()
#        #self.list_widget.addItem(name)
#        self.head_angs_list.append({'name':name, 'time': self.time_box.value(), 'angs': self._read_head_position_from_fields()})
#        self._refill_list_widget(self.head_angs_list)
#
##added
#    def save_button_cb(self):
#        idx = self._selected_idx()
#        if idx == None:
#            return
#        el = self.head_angs_list[idx]
#        self.head_angs_list[idx] = {'name': el['name'],
#            'time': self.time_box.value(), 
#            'angs': self._read_head_position_from_fields()}
#
##added
#    def _read_head_position_from_fields(self):
#        headPos = []
#        for name in self.joint_names:
#            #exec('rad = np.radians(float(str(self.%s.text())))' % name)
#            exec('rad = np.radians(self.%s.value())' % name)
#            headPos.append(rad)
#        return headPos
#
##added
#    def remove_pose_cb(self):
#        idx = self._selected_idx()
#        if idx == None:
#            return
#        #self.list_widget.takeItem(idx)
#        #self.list_widget.removeItemWidget(selected[0])
#        if idx == None:
#            raise RuntimeError('Inconsistency detected in list')
#        else:
#            self.head_angs_list.pop(idx)
#        self._refill_list_widget(self.head_angs_list)
#
##added
#    def _create_name(self):
#        idx = len(self.head_angs_list)
#        tentative_name = 'point%d' % idx 
#
#        while self._has_name(tentative_name):
#            idx = idx + 1
#            tentative_name = 'point%d' % idx 
#
#        return tentative_name
#
##added
#    def _selected_idx(self):
#        #Get currently selected
#        selected = self.list_widget.selectedItems()
#        if len(selected) == 0:
#            return None
#        sname = str(selected[0].text())
#
#        #Remove it from list_widget and head_angs_list
#        idx = self._find_index_of(sname)
#        return idx
#
##added
#    def _find_index_of(self, name):
#        for idx, tup in enumerate(self.head_angs_list):
#            if tup['name'] == name:
#                return idx
#        return None
#
#    def reset(self):
#        for box in self.joint_boxes:
#            box.setValue(0)
#        self.time_box.setValue(1.)
#	self.update_checkbox.setCheckState(False)
#        self.status_bar_timer.stop()
#        self.current_pose_button.setEnabled(True)
#
#    def new_node(self, name=None):
#        if name == None:
#            nname = self.name + str(self.counter)
#        else:
#            nname = name
#
#        poses = np.matrix([np.radians(self.joint_boxes[0].value()),
#			   np.radians(self.joint_boxes[1].value())]).T
#        return MoveHeadState(nname, poses, self.head_angs_list)
#
##added
#    def save_button_cb(self):
#        idx = self._selected_idx()
#        if idx == None:
#            return
#        el = self.head_angs_list[idx]
#        self.head_angs_list[idx] = {'name': el['name'],
#            'time': self.time_box.value(), 
#            'angs': self._read_head_position_from_fields()}
##added
#    def update_selected_cb(self, state):
#        # checked
#        if state == 2:
#            self.status_bar_timer.start(30)
#            self.current_pose_button.setEnabled(False)
#	    self.get_current_joint_angles_cb()
#        # unchecked
#        if state == 0:
#            self.status_bar_timer.stop()
#            self.current_pose_button.setEnabled(True)
#
##added
#    def move_up_cb(self):
#        #get the current index
#        idx = self._selected_idx()
#        if idx == None:
#            return
#
#        #pop & insert it
#        item = self.head_angs_list.pop(idx)
#        self.head_angs_list.insert(idx-1, item)
#
#        #refresh
#        self._refill_list_widget(self.head_angs_list)
#        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))
##added
#    def get_current_joint_angles(self):
#        pose_mat = rcommander.robot.head.pose()
#
#        for idx, name in enumerate(self.joint_name_fields):
#            deg = np.degrees(pose_mat[idx, 0])
#            exec('line_edit = self.%s' % name)
#            #line_edit.setText('%.2f' % deg)
#            line_edit.setValue(deg)
#
#    def move_down_cb(self):
#        #get the current index
#        idx = self._selected_idx()
#        if idx == None:
#            return
#
#        #pop & insert it
#        item = self.head_angs_list.pop(idx)
#        self.head_angs_list.insert(idx+1, item)
#
#        #refresh
#        self._refill_list_widget(self.head_angs_list)
#        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))












