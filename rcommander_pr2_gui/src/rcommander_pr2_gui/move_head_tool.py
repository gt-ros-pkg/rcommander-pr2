import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import actionlib_msgs.msg as am
import numpy as np
import smach

class MoveHeadTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_head_ang', 'Move Head (ang)', MoveHeadState)
	self.head_angs_list = None
	self.joint_names = []
	self.status_bar_timer = QTimer()
	self.rcommander.connect(self.status_bar_timer, SIGNAL('timeout()'), self.get_current_joint_angles_cb)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
	
        joint_names = self.rcommander.robot.head.get_joint_names()
        self.joint_boxes = []
        for jname in joint_names:
            joint_box = QDoubleSpinBox(pbox)
            joint_box.setMinimum(-9999.)
            joint_box.setMaximum(9999.)
            joint_box.setSingleStep(1.)
            self.joint_boxes.append(joint_box)
            formlayout.addRow('&%s' % jname, joint_box)

	self.head_angs_list = []
        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.5)
        formlayout.addRow('&Time', self.time_box)

	self.update_checkbox = QCheckBox(pbox) 
        self.update_checkbox.setTristate(False)
        formlayout.addRow('&Live Update', self.update_checkbox)
 	self.rcommander.connect(self.update_checkbox, SIGNAL('stateChanged(int)'), self.update_selected_cb)

        self.current_pose_button = QPushButton(pbox)
        self.current_pose_button.setText('Current Pose')
        self.rcommander.connect(self.current_pose_button, 
                SIGNAL('clicked()'), self.get_current_joint_angles_cb)
        formlayout.addRow('    ', self.current_pose_button)
     
#widet box 
	self.list_box = QWidget(pbox)
        self.list_box_layout = QHBoxLayout(self.list_box)
        self.list_box_layout.setMargin(0)

        self.list_widget = QListWidget(self.list_box)
        self.rcommander.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)
        self.list_box_layout.addWidget(self.list_widget)

        #self.movement_buttons_widget = QWidget(self.list_box)
        #self.movement_buttons_widgetl = QVBoxLayout(self.movement_buttons_widget)
        #self.movement_buttons_widgetl.setMargin(0)

        self.list_widget_buttons = QWidget(pbox)
        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)

        self.move_up_button = QPushButton(self.list_widget_buttons)
        self.move_up_button.setText('Up')
        self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
        self.lbb_hlayout.addWidget(self.move_up_button)

        self.move_down_button = QPushButton(self.list_widget_buttons)
        self.move_down_button.setText('Down')
        self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
        self.lbb_hlayout.addWidget(self.move_down_button)
	#self.lbb_hlayout.addSpacing(50)

        spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.lbb_hlayout.addItem(spacer)

	#formlayout.addRow('&Create a Joint Sequence', self.time_box)
	   
        self.add_head_set_button = QPushButton(self.list_widget_buttons)
        self.add_head_set_button.setText('Add Head Position')
        self.rcommander.connect(self.add_head_set_button, SIGNAL('clicked()'), self.add_head_set_cb)

        self.remove_head_set_button = QPushButton(self.list_widget_buttons)
        self.remove_head_set_button.setText('Remove Head Position')
        self.rcommander.connect(self.remove_head_set_button, SIGNAL('clicked()'), self.remove_pose_cb)
        self.save_button = QPushButton(self.list_widget_buttons)
        self.save_button.setText('Save Sequence')
        self.rcommander.connect(self.save_button, SIGNAL('clicked()'), self.save_button_cb)

        self.lbb_hlayout.addWidget(self.add_head_set_button)
        self.lbb_hlayout.addWidget(self.remove_head_set_button)
        self.lbb_hlayout.addWidget(self.save_button)
        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)


	#FIX	 
	formlayout.addRow('\n', self.list_box)      
	formlayout.addRow('&Create a Head Movement Sequence:', self.list_box)
	formlayout.addRow(self.list_box)
        formlayout.addRow(self.list_widget_buttons)


        self.reset()

    def _refill_list_widget(self, joints_list):
        self.list_widget.clear()
        for d in joints_list:
            self.list_widget.addItem(d['name'])

    def get_current_joint_angles_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.joint_boxes[0].setValue(np.degrees(poses[0,0]))
	print 'first one'
	print np.degrees(poses[0,0])
        self.joint_boxes[1].setValue(np.degrees(poses[1,0]))
	print 'second one'
	print np.degrees(poses[1,0])

    def set_node_properties(self, my_node):
        self.joint_boxes[0].setValue(np.degrees(my_node.poses[0,0]))
        self.joint_boxes[1].setValue(np.degrees(my_node.poses[1,0]))
        self.time_box.setValue(my_node.mot_time)
#added   
    def item_selection_changed_cb(self):
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return
        idx = self._find_index_of(str(selected[0].text()))
        head_angs = self.head_angs_list[idx]['angs']
        self._set_joints_to_fields(head_angs)
        self.time_box.setValue(self.head_angs_list[idx]['time'])
#added
    def _set_joints_to_fields(self, joints):
        for idx, name in enumerate(self.joint_names):
            deg = np.degrees(joints[idx])
            exec('line_edit = self.%s' % name)
            line_edit.setValue(deg)

#added
    def _has_name(self, test_name):
        for rec in self.head_angs_list:
            if rec['name'] == test_name:
                return True
            else:
                return False

#added
    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.head_angs_list.pop(idx)
        self.head_angs_list.insert(idx-1, item)

        #refresh
        self._refill_list_widget(self.head_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))

#added
    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.head_angs_list.pop(idx)
        self.head_angs_list.insert(idx+1, item)

        #refresh
        self._refill_list_widget(self.head_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))

#added
    def add_head_set_cb(self):
        #Create a new string, check to see whether it's in the current list
        name = self._create_name()
        #self.list_widget.addItem(name)
        self.head_angs_list.append({'name':name, 'time': self.time_box.value(), 'angs': self._read_head_position_from_fields()})
        self._refill_list_widget(self.head_angs_list)

#added
    def save_button_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        el = self.head_angs_list[idx]
        self.head_angs_list[idx] = {'name': el['name'],
            'time': self.time_box.value(), 
            'angs': self._read_head_position_from_fields()}

#added
    def _read_head_position_from_fields(self):
        headPos = []
        for name in self.joint_names:
            #exec('rad = np.radians(float(str(self.%s.text())))' % name)
            exec('rad = np.radians(self.%s.value())' % name)
            headPos.append(rad)
	    print name
        return headPos

#added
    def remove_pose_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        #self.list_widget.takeItem(idx)
        #self.list_widget.removeItemWidget(selected[0])
        if idx == None:
            raise RuntimeError('Inconsistency detected in list')
        else:
            self.head_angs_list.pop(idx)
        self._refill_list_widget(self.head_angs_list)

#added
    def _create_name(self):
        idx = len(self.head_angs_list)
        tentative_name = 'point%d' % idx 

        while self._has_name(tentative_name):
            idx = idx + 1
            tentative_name = 'point%d' % idx 

        return tentative_name

#added
    def _selected_idx(self):
        #Get currently selected
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return None
        sname = str(selected[0].text())

        #Remove it from list_widget and head_angs_list
        idx = self._find_index_of(sname)
        return idx

#added
    def _find_index_of(self, name):
        for idx, tup in enumerate(self.head_angs_list):
            if tup['name'] == name:
                return idx
        return None

    def reset(self):
        for box in self.joint_boxes:
            box.setValue(0)
        self.time_box.setValue(1.)
	self.update_checkbox.setCheckState(False)
        self.status_bar_timer.stop()
        self.current_pose_button.setEnabled(True)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        poses = np.matrix([np.radians(self.joint_boxes[0].value()),
			   np.radians(self.joint_boxes[1].value())]).T
        return MoveHeadState(nname, poses, self.time_box.value())

#added
    def save_button_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        el = self.joint_angs_list[idx]
        self.joint_angs_list[idx] = {'name': el['name'],
            'time': self.time_box.value(), 
            'angs': self._read_joints_from_fields()}
#added
    def update_selected_cb(self, state):
        # checked
        if state == 2:
            self.status_bar_timer.start(30)
            self.current_pose_button.setEnabled(False)
	    self.get_current_joint_angles_cb()
	    print 'getting the current joint angles'
        # unchecked
        if state == 0:
            self.status_bar_timer.stop()
            self.current_pose_button.setEnabled(True)

#added
    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.head_angs_list.pop(idx)
        self.head_angs_list.insert(idx-1, item)

        #refresh
        self._refill_list_widget(self.head_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))
#added
    def get_current_joint_angles(self):
        pose_mat = rcommander.robot.head.pose()

        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(pose_mat[idx, 0])
            exec('line_edit = self.%s' % name)
            #line_edit.setText('%.2f' % deg)
            line_edit.setValue(deg)

    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.head_angs_list.pop(idx)
        self.head_angs_list.insert(idx+1, item)

        #refresh
        self._refill_list_widget(self.head_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))


class MoveHeadState(tu.StateBase):

    def __init__(self, name, poses, mot_time):
        tu.StateBase.__init__(self, name)
        self.poses = poses 
        self.mot_time = mot_time

    def get_smach_state(self):
        return MoveHeadStateSmach(self.poses, self.mot_time)


class MoveHeadStateSmach(smach.State):

    def __init__(self, poses, mot_time):
        smach.State.__init__(self, outcomes=['preempted', 'done'], 
			     input_keys=[], output_keys=[])
        self.mot_time = mot_time
        self.poses = poses
        self.robot = None

    def set_robot(self, robot):
        if robot != None:
            self.head_obj = robot.head

    def execute(self, userdata):
        #self.head_obj.set_poses(self,self.poses, self.mot_time)
	self.head_obj.set_pose(self.poses, self.mot_time)
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.mot_time:
            r.sleep()
            if self.preempt_requested():
                self.services_preempt()
                return 'preempted'
        return 'done'











