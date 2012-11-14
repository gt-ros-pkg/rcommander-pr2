## Classes for construction of PR2 ROS Commander GUIs and interfacing
# with builtin PR2 actions.
# TODO: refactor and separate GUI bits from PR2 bits

import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rospy
import actionlib
import actionlib_msgs.msg as amsg
import trajectory_msgs.msg as tm
import numpy as np
import functools as ft
import sensor_msgs.msg as sm
import std_msgs.msg as stdm
import pr2_controllers_msgs.msg as pm
import move_base_msgs.msg as mm
import geometry_msgs.msg as gm
import geometry_msgs.msg as geo
import time
from kinematics_msgs.srv import GetKinematicSolverInfo
from pycontroller_manager.pycontroller_manager import ControllerManager
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tf_utils as tfu
from object_manipulator.convert_functions import stamp_pose
import tf.transformations as tr
from tf_broadcast_server.srv import GetTransforms
import rcommander.tool_utils as tu
import roslib
import os.path as pt
import copy
import unittest
import math

import pypr2.msg as hm #TODO move this somewhere

## PR2 joint names
JOINT_NAME_FIELDS = ["shoulder_pan_joint", "shoulder_lift_joint", 
                    "upper_arm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint", 
                    "wrist_roll_joint"]

## Translation of joint names to more readable strings
HUMAN_JOINT_NAMES = ['Shoulder Pan', 'Shoulder Lift', 'Uper Arm Roll', 
                    'Elbow Flex', 'Forearm Roll', 'Wrist Flex', 'Wrist Roll']


## Create a QPalette from rgb + alpha values
def create_color(a, r, g, b):
    palette = QPalette(QColor(a, r, g, b))
    palette.setColor(QPalette.Text, QColor(a, r, g, b))
    return palette


## Mixin class for classes implementing live update functionality.
class LiveUpdateTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander, name, button_name, smach_class):
        tu.ToolBase.__init__(self, rcommander, name, button_name, 
                smach_class)

        self.live_update = False
        self.live_update_timer = QTimer()
        self.current_update_color = (0,0,0,255)
        rcommander.connect(self.live_update_timer, SIGNAL('timeout()'), 
                self.live_update_cb)

    ## Create live update button, needs to be called by inherited classes
    # @param qt_container A QT Container where the live update button can be
    #                       inserted.
    def create_live_update_controls(self, qt_container, connector):
        self.live_update_button = QPushButton(qt_container)
        self.live_update_button.setText('Live Update')
        connector.connect(self.live_update_button, SIGNAL('clicked()'), 
                self.live_update_button_cb)
        qt_container.layout().addWidget(self.live_update_button)

    ## Callback for live update button
    def live_update_button_cb(self):
        if self.live_update_button.text() == 'Live Update':
            self.set_update_mode(True)
        else:
            self.set_update_mode(False)

    ## Sets the update mode of dialog box 
    # @param on Whether the update mode should be turned on (bool)
    def set_update_mode(self, on):
        self.live_update = on

        if on:
            self.live_update_toggle_cb(self.live_update)

        if self.live_update:
            self.live_update_button.setText('End Live Update')
            self.live_update_button.setEnabled(True)
            self.live_update_timer.start(30)
            self.current_update_color = (0,180,75,255)
        else:
            self.live_update_button.setText('Live Update')
            self.live_update_timer.stop()
            self.current_update_color = (0,0,0,255)

        palette = create_color(*self.current_update_color)
        for name in self.get_colorable_fields():      
            exec('self.%s.setPalette(palette)' % name)

        if not on:
            self.live_update_toggle_cb(self.live_update)

    ## Override ToolBase's deselect
    def deselect_tool(self):
        self.set_update_mode(False)
        tu.ToolBase.deselect_tool(self)

    ## Function that inherited classes must implement.
    # Has to return the name (as a string) of fields in class that
    # should be colored green when live update turns on.
    def get_colorable_fields(self):
        return []

    ## Function that inherited classes must implement.
    # Called whenever update timer fires
    def live_update_cb(self):
        pass

    ## Function that inherited classes must implement.
    # Called whenever the live update button is toggled.
    # @param state Current state of live update button (bool).
    def live_update_toggle_cb(self, state):
        pass

## Like LiveUpdate but manages interaction between the live update
# mode and editing in the ListManager
class LiveUpdateListTool(LiveUpdateTool):

    ## Constructor
    # @param rcommander RCommander class
    # @param name Name of tool.
    # @param button_name Name of button for tool.
    # @param smach_class Class of SMACH nodes produced by this tool.
    def __init__(self, rcommander, name, button_name, smach_class):
        LiveUpdateTool.__init__(self, rcommander, name, button_name, 
                smach_class)
        self.reset_live_update = True
        self.element_will_be_added = False

    ## Inherited classes should call this to create QT GUI widgets
    # to handle live update and creating lists.
    # @param live_update_container QWidget container to add widgets to.
    # @param name_preffix Prefix for entries created in ListManager.
    def make_live_update_gui_elements(self, live_update_container, 
            name_preffix='point'):
        self.create_live_update_controls(live_update_container, 
                self.rcommander)
        self.list_manager = ListManager(\
                self.get_current_data_from_gui_fields_cb, 
                self._set_current_data_to_gui_fields, 
                self.list_manager_add_element_cb, 
                name_preffix=name_preffix)
        return self.list_manager

    ## Callback when list manager add an element
    def list_manager_add_element_cb(self):
        self.element_will_be_added = True

    ## Callback when live update timer fire
    def live_update_cb(self):
        self.reset_live_update = False
        data = self.get_data_update_from_robot_cb()
        if data != None:
            self.list_manager.display_record(data)
        self.reset_live_update = True

    ## Inherited from JointTool
    def live_update_toggle_cb(self, state):
        if not state:
            self.reset_live_update = False
            selected_name = self.list_manager.get_selected_name()
            if selected_name != None:
                self.list_manager.set_selected_by_name(selected_name)
            self.reset_live_update = True

    ## Callback from ListManager to set GUI data fields (internal version)
    def _set_current_data_to_gui_fields(self, data):
        if self.reset_live_update and not self.element_will_be_added:
            # - This is here so that when user selects another point
            # live update is turned OFF but add also uses this
            # function, which turns OFF live update!
            # - The toggle button function only remembers the element
            # activated when it was clicked.
            self.set_update_mode(False) 

        self.set_current_data_to_gui_fields_cb(data)

        if self.element_will_be_added:
            self.element_will_be_added = False

    ########################################################################
    ## Everything below has to be implemented by inheriting class (see
    # def. in LiveUpdateTool).
    ## Return the name of variables defined in this class that 
    # represent data fields that can be colored.
    def get_colorable_fields(self):
        return []

    ## Has to be implemented to use list manager (see def. for
    # get_current_data_cb in ListManager)
    def get_current_data_from_gui_fields_cb(self):
        pass

    ## Has to be implemented by inheriting class.
    # Must return the same ListManager list format as get_current_data_cb
    # (see ListManager for ref).
    def get_data_update_from_robot_cb(self):
        pass

    ## Has to be implemented to use list manager (see def. for
    # set_current_data_cb in ListManager)
    def set_current_data_to_gui_fields_cb(self, data):
        pass

    ## Has to be implemented and called by inheriting class (part of ToolBase)
    def new_node(self, name=None):
        self.list_manager.save_currently_selected_item()

    ## Has to be implemented by inheriting class (part of ToolBase)
    def set_node_properties(self, my_node):
        pass

    ## Has to be implemented and called by inheriting class (part of ToolBase)
    def reset(self):
        self.list_manager.reset()
        self.list_manager.set_default_selection()
        self.live_update_timer.stop()

## Mixin for classes implementing ToolBase to display/edit PR2 joint angles
class JointTool:

    ## Constructor
    # @param robot PR2 robot object used by RCommander.
    # @param rcommander RCommander instance.
    def __init__(self, robot, rcommander):
        self.limits = [robot.left.get_limits(), robot.right.get_limits()]
        self.robot = robot

    ## Set the arm radio buttons
    # @param arm either 'left' or 'right'
    def set_arm_radio(self, arm):
        if 'left' == arm:
            self.arm_radio_buttons[0].setChecked(True)
        if arm == 'right':
            self.arm_radio_buttons[1].setChecked(True)

    ## Get the value of the arm radio buttons
    # @return either 'left' or 'right'
    def get_arm_radio(self):
        return tu.selected_radio_button(self.arm_radio_buttons).lower()

    ## Get the radio button classes in a list of size 2
    def get_arm_radio_buttons(self):
        return self.arm_radio_buttons

    ## Reset all joint display fields
    def set_all_fields_to_zero(self):
        for name in JOINT_NAME_FIELDS:
            exec('self.%s.setValue(0)' % name)

    ## Make a QDoubleSpinBoxe for each joint
    # @param pbox Properties panel in RCommander (a QWidget)
    def make_joint_boxes(self, pbox):
        fields = []
        formlayout = pbox.layout()

        self.arm_radio_boxes, self.arm_radio_buttons = tu.make_radio_box(pbox, 
                ['Left', 'Right'], 'arm')
        for name, friendly_name in zip(JOINT_NAME_FIELDS, HUMAN_JOINT_NAMES):
            exec("self.%s = QDoubleSpinBox(pbox)" % name)
            exec('box = self.%s' % name)
            box.setSingleStep(.5)
            box.setMinimum(-9999999)
            box.setMaximum(9999999)
            fields.append({"name": "&%s" % friendly_name, 
                           "item": box, 
                           'joint': name})

        return fields, self.arm_radio_boxes, []

    ## Update the data fields
    # @param joints A list of 7 doubles.
    def set_joints_to_fields(self, joints):
        for idx, name in enumerate(JOINT_NAME_FIELDS):
            deg = np.degrees(joints[idx])
            exec('line_edit = self.%s' % name)
            line_edit.setValue(deg)

    ## Read joint angles from data fields.
    # @param limit_ranges Whether to keep angles within a certain range.
    # @return List of joint angles (7)
    def read_joints_from_fields(self, limit_ranges=False):
        arm = self.get_arm_radio()
        if arm == 'left':
            idx = 0
            pref = 'l_'
        else:
            idx = 1
            pref = 'r_'

        limits = self.limits[idx]
        joints = []
        for name in JOINT_NAME_FIELDS:
            exec('rad = np.radians(self.%s.value())' % name)
            if limit_ranges and limits.has_key(pref+name):
                mn, mx = limits[pref+name]
                nrad = max(min(rad, mx-.005), mn+.005)
                rad = nrad
            joints.append(rad)
        return joints

    ## Get the current joint angles from the robot
    # @return pose_mat 7x1 numpy matrix
    def get_robot_joint_angles(self):
        arm = self.get_arm_radio()
        if ('left' == arm):
            arm_obj = self.robot.left
        else:
            arm_obj = self.robot.right
        pose_mat = arm_obj.pose()
        pose_mat[4,0] = pose_mat[4,0] % (np.pi*2)
        pose_mat[6,0] = pose_mat[6,0] % (np.pi*2)

        return pose_mat

    ## Get the current joint angles from the robot and set them in the
    # QDoubleSpinBoxes.
    def current_pose_cb(self):
        pose_mat = self.get_robot_joint_angles()
        for idx, name in enumerate(JOINT_NAME_FIELDS):
            deg = np.degrees(pose_mat[idx, 0])
            exec('line_edit = self.%s' % name)
            line_edit.setValue(deg)

    ##  Check the values entered by users against the robot's joint limits.
    # @param valid_color Color of fields that are valid (as a tuple of
    #                   4 floats for rgb+a).
    def check_all_joint_limits(self, valid_color):
        arm = self.get_arm_radio()
        for idx, name in enumerate(JOINT_NAME_FIELDS):
            exec('line_edit = self.%s' % name)
            self.check_joint_limits(arm, line_edit.value(), name, valid_color)

    ## 
    # Check the joint limit for a single joint QDoubleSpinBox
    # @param arm Which arm ('left' or 'right').
    # @param value Joint angle (radians, float).
    # @param joint Joint name (string.
    # @param valid_color Color of fields that are alid (as tuple of 4
    #                   floats for rgb+a)
    def check_joint_limits(self, arm, value, joint, valid_color):
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
                self.set_invalid_color(joint, True, valid_color)
            else:
                self.set_invalid_color(joint, False, valid_color)

    ## Set the color of QDoubleSpinBox
    # @param joint_name name of joint (string)
    # @param invalid whether this joint spin box is invalid or not (bool)
    # @param valid_color list of 4 floats for rgb+a
    # @param invalid_color list of 4 floats for rgb+a
    def set_invalid_color(self, joint_name, invalid, valid_color, 
            invalid_color = [255,0,0,255]):
        if invalid:
            palette = create_color(*invalid_color)
        else:
            palette = create_color(*valid_color)
        exec('self.%s.setPalette(palette)' % joint_name)


## Make a drop down that allows frame selection
def make_frame_box(pbox, frames_service):
    frame_box = QComboBox(pbox)
    for i in range(3):
        try:
            frames = frames_service().frames
            break
        except AttributeError, e:
            frames_service = rospy.ServiceProxy('get_transforms', 
                    GetTransforms, persistent=True)
    for f in frames:
        frame_box.addItem(f)
    return frame_box, frames_service

## Mixin for tools that want text boxes for editing values that exist in SE3
class SE3Tool:

    ## Constructor
    def __init__(self):
        self.frames_service = rospy.ServiceProxy('get_transforms', 
                GetTransforms, persistent=True)

    ## Should be called by children to make GUI widgets
    # @return A list with two widgets that should be inserted into an
    #           appropriate container.
    def make_se3_boxes(self, pbox):
        self.xline = tu.double_spin_box(pbox, -200., 200.,.01)
        self.yline = tu.double_spin_box(pbox, -200., 200.,.01)
        self.zline = tu.double_spin_box(pbox, -200., 200.,.01)
        self.phi_line   = tu.double_spin_box(pbox, -360., 360., 1)
        self.theta_line = tu.double_spin_box(pbox, -360., 360., 1)
        self.psi_line   = tu.double_spin_box(pbox, -360., 360., 1)
    
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

    ## Gets all data input widgets
    # @return a list of QDoubleSpinBoxes
    def get_all_data_input_widgets(self):
        return [self.xline, self.yline, self.zline, 
                self.phi_line, self.theta_line, self.psi_line]
    
    ## Create a task frame drop down 
    def make_task_frame_box(self, pbox):
        self.frame_box, self.frames_service = \
                make_frame_box(pbox, self.frames_service)

        return self.frame_box

    ## Get data contained in text fields as a PoseStamped 
    def get_posestamped(self):
        pose  = geo.Pose()
        pose.position = geo.Point(*[float(vr.value()) \
                for vr in [self.xline, self.yline, self.zline]])
        pose.orientation = geo.Quaternion(*tr.quaternion_from_euler(\
                *[float(np.radians(vr.value())) for vr in \
                [self.phi_line, self.theta_line, self.psi_line]]))
        ps = stamp_pose(pose, str(self.frame_box.currentText()))
        return ps

    ## Set the text fields as a PoseStamped
    def set_posestamped(self, pose_stamped):
        for value, vr in zip(position(pose_stamped.pose.position), \
                [self.xline, self.yline, self.zline]):
            vr.setValue(value)
        for value, vr in zip(tr.euler_from_quaternion(quaternion(\
                pose_stamped.pose.orientation)),\
                [self.phi_line, self.theta_line, self.psi_line]):
            vr.setValue(np.degrees(value))
        idx = tu.combobox_idx(self.frame_box, pose_stamped.header.frame_id)
        self.frame_box.setCurrentIndex(idx)


## Creates a list box with controls for adding, removing, sorting items.
class ListManager:

    ## Constructor
    # @param get_current_data_cb Callback when this ListManager need
    #                               to get data.
    # @param set_current_data_cb Callback this ListManager need to
    #                               set data.
    # @param add_element_cb Callback when an element gets added to list.
    # @param name_preffix Preffix of elements in list.
    def __init__(self, get_current_data_cb, set_current_data_cb, 
            add_element_cb=None, name_preffix='point'):
        self.get_current_data_cb = get_current_data_cb
        self.set_current_data_cb = set_current_data_cb
        self.add_element_cb = add_element_cb
        self.name_preffix = name_preffix
        self.data_list = []
        self.curr_selected = None
        self.disable_saving = False
        self.displayed_record = False

    ## Clear list data
    def reset(self):
        self.curr_selected = None
        self.disable_saving = False
        self.data_list = []
        self.list_widget.clear()

    ## Callback for when item selection changed
    def item_selection_changed_cb(self):
        if self.curr_selected != None:
            self.save_currently_selected_item()

        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return

        selected_name = str(selected[0].text())
        self.set_selected_by_name(selected_name)

    def get_selected_idx(self):
        return self.curr_selected

    ## Get name of currently selected item
    def get_selected_name(self):
        selected = self.list_widget.selectedItems() 
        if len(selected) == 0:
            return None
        name_selected = str(selected[0].text())
        return name_selected

    ## Set the selected element by that element's name
    # @param name 'string'
    def set_selected_by_name(self, name):
        idx = self._find_index_of(name)
        self.curr_selected = idx
        self.set_current_data_cb(self.data_list[idx]['data'])
        self.displayed_record = False

    ## Set the default selected item.
    def set_default_selection(self):
        if len(self.data_list) > 0:
            self.list_widget.setCurrentItem(0)

    ## Used to do a reverse lookup frome name to index in data list
    # @param name Name of element changed
    def _find_index_of(self, name):
        for idx, tup in enumerate(self.data_list):
            if tup['name'] == name:
                return idx
        return None

    def monitor_changing_values_in(self, connector, widgets):
        def value_changed_func(new_value):
            self.save_currently_selected_item()
        for w in widgets:
            connector.connect(w, SIGNAL('valueChanged(double)'), 
                    value_changed_func)

    
    ## Called by inheritted classes make QTWidgets
    # @param parent Parent QWidget.
    # @param connector Object that will understand QT signasl
    def make_widgets(self, parent, connector):
        self.list_box = QWidget(parent)
        self.list_box_layout = QHBoxLayout(self.list_box)
        self.list_box_layout.setMargin(0)

        self.list_widget = QListWidget(self.list_box)
        connector.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), 
                self.item_selection_changed_cb)
        self.list_box_layout.addWidget(self.list_widget)

        self.list_widget_buttons = QWidget(parent)
        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)
        base_path = roslib.packages.get_pkg_dir('rcommander_pr2_gui')

        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/AddButton.png")), 
                QIcon.Normal, QIcon.Off)
        self.add_button = QPushButton(self.list_widget_buttons)
        self.add_button.setToolTip('Add')
        self.add_button.setIcon(icon)
        connector.connect(self.add_button, SIGNAL('clicked()'), self.add_cb)

        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/RemoveButton.png")), 
                QIcon.Normal, QIcon.Off)
        self.remove_button = QPushButton(self.list_widget_buttons)
        self.remove_button.setToolTip('Remove')
        self.remove_button.setIcon(icon)
        connector.connect(self.remove_button, SIGNAL('clicked()'), 
                self.remove_cb)

        spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, 
                QSizePolicy.Expanding)

        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/UpButton.png")), 
                QIcon.Normal, QIcon.Off)
        self.move_up_button = QPushButton(self.list_widget_buttons)
        self.move_up_button.setToolTip('Up')
        self.move_up_button.setIcon(icon)
        connector.connect(self.move_up_button, SIGNAL('clicked()'), 
                self.move_up_cb)

        icon = QIcon()
        icon.addPixmap(QPixmap(pt.join(base_path, "icons/DownButton.png")), 
                QIcon.Normal, QIcon.Off)
        self.move_down_button = QPushButton(self.list_widget_buttons)
        self.move_down_button.setToolTip('Down')
        self.move_down_button.setIcon(icon)
        connector.connect(self.move_down_button, SIGNAL('clicked()'), 
                self.move_down_cb)

        self.lbb_hlayout.addWidget(self.add_button)
        self.lbb_hlayout.addWidget(self.remove_button)
        self.lbb_hlayout.addItem(spacer)
        self.lbb_hlayout.addWidget(self.move_up_button)
        self.lbb_hlayout.addWidget(self.move_down_button)
        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)
        return [self.list_box, self.list_widget_buttons]


    ## Refresh display to match given data_list
    # @param data_list a list of dictionaries with items of the format
    # {'name': string} where each is a 'data item' displayed.
    def _refill_list_widget(self, data_list):
        self.list_widget.clear()
        for d in data_list:
            self.list_widget.addItem(d['name'])

    ## Check to see the given name exist in our data list
    # @return boolean
    def _has_name(self, test_name):
        for rec in self.data_list:
            if rec['name'] == test_name:
                return True
        return False

    ## Create a default name for new items in the data list
    # @return a name (string)
    def _create_name(self):
        idx = len(self.data_list)
        tentative_name = self.name_preffix + ('%d' % idx)
        while self._has_name(tentative_name):
            idx = idx + 1
            tentative_name = self.name_preffix + ('%d' % idx)
        return tentative_name

    ## Callback for add button (green plus).  Adds whatever is being
    # displayed by inherited class.
    def add_cb(self):
        name = self._create_name()
        self.data_list.append({'name': name, 
                               'data': self.get_current_data_cb()})
        self.disable_saving = True
        if self.add_element_cb != None:
            self.add_element_cb()
        self._refill_list_widget(self.data_list)
        self.list_widget.setCurrentRow(self._find_index_of(name))
        self.disable_saving = False

    ## Display a data record in GUI data fields
    # @param data_record
    def display_record(self, data_record):
        self.disable_saving = True
        self.set_current_data_cb(data_record)
        self.displayed_record = True
        self.disable_saving = False

    ## Gets the index of the currently selected element
    # @return an integer
    def _selected_idx(self):
        #Get currently selected
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return None
        sname = str(selected[0].text())

        idx = self._find_index_of(sname)
        return idx
    
    ## Callback: moves a data record up the list in data display
    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.data_list.pop(idx)
        self.data_list.insert(idx-1, item)

        #refresh
        self.disable_saving = True
        self._refill_list_widget(self.data_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))
        self.disable_saving = False

    ## Callback: moves a data record down the list in data display
    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.data_list.pop(idx)
        self.data_list.insert(idx+1, item)

        #refresh
        self.disable_saving = True
        self._refill_list_widget(self.data_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))
        self.disable_saving = False

    ## Set the selection in display to the default item
    def select_default_item(self):
        self.list_widget.setCurrentItem(self.list_widget.item(0))

    ## Save currently selected item
    def save_currently_selected_item(self):
        if self.disable_saving or self.displayed_record:
            return

        idx = self.curr_selected
        if idx == None or idx >= len(self.data_list):
            return
        el = self.data_list[idx]
        self.data_list[idx] = {'name': el['name'],
                               'data': self.get_current_data_cb()}

    ## Callback: remove selected item
    def remove_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        if idx == None:
            raise RuntimeError('Inconsistency detected in list')

        self.disable_saving = True
        self.data_list.pop(idx)
        self._refill_list_widget(self.data_list)
        self.curr_selected = None
        self.disable_saving = False

    ## Get data being managed by this display list
    # @return a list of data dictionaries of the form {'name': x,
    # 'data': y} where x is the name of the item and y is the stored}
    def get_data(self, clean=False):
        if clean:
            return [d['data'] for d in self.data_list]
        else:
            return self.data_list

    ## Set data being managed by this display list
    # @param data_list a list of data dictionaries of the form {'name': x,
    # 'data': y} where x is the name of the item and y is the stored}
    def set_data(self, data_list):
        self.data_list = copy.deepcopy(data_list)
        self._refill_list_widget(self.data_list)

## Takes a ROS position and construct a list of 3 numbers from its xyz fields
def position(point):
    return [point.x, point.y, point.z]


## Takes a ROS quarternion and construct a list of 4 numbers from its
# xyzw fields
def quaternion(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

#def unwrap2(cpos, npos):
#    two_pi = 2*np.pi
#    nin = npos % two_pi
#    n_multiples_2pi = np.floor(cpos/two_pi)
#    return nin + n_multiples_2pi*two_pi

## Takes the minimum representation of an angular quantity
# @param t angle in radian (float)
def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi

## Used to make sure the PR2 arm doesn't spin when sent a list of 
# joint angles since there are continuous rotation joints.
# Make sure that the first element in angles list is close to the
# current angle and make sure that every other element conforms.
# @param current_angle Angle in radians.
# @param angles_list List of angles in radians.
# @param allow_spins Whether to allow the arm to spin (boolean).
def angle_consistency_check(current_angle, angles_list, allow_spins=False):
    diff = standard_rad(angles_list[0] - current_angle)
    start_angle = current_angle + diff
    angles = []
    prev = angles_list[0]
    for current in angles_list[1:]:
        if allow_spins:
            angles.append(start_angle + (current - prev))
        else:
            angles.append(start_angle + standard_rad(current - prev))

    return [start_angle] + angles

## Unit test for the above function.
ra = math.radians
class TestAngleConsistency(unittest.TestCase):

    def test_01(self):
        sa = ra(360 + 10.)
        la = [ra(20.), ra(360+180.)]
        ret = angle_consistency_check(sa, la)
        self.assertEqual(ret, [ra(360+20.), ra(360.+360.+180)])

    def test_02(self):
        sa = ra(10 - 360)
        la = [ra(20.), ra(360+180.)]
        ret = angle_consistency_check(sa, la)
        self.assertEqual(ret, [ra(20.-360), ra(-360.+360.+180)])

    def test_03(self):
        ret = angle_consistency_check(0, [ara(3620), ra(3960), ra(4320)])
        self.assertEqual(ret, [ra(20.), ra(360.), ra(720.)])

    def test_04(self):
        ret = angle_consistency_check(ra(-3600.), [ra(0), ra(10), ra(360)])
        self.assertEqual(ret, [ra(-3600.), ra(-3590.), ra(-3240.)])


##
# Takes a normal ROS callback channel and gives it an on demand query style
# interface.
class GenericListener:
    ##
    # Message has to have a header
    # @param node_name name of node (if haven't been inited)
    # @param message_type type of message to listen for
    # @param listen_channel ROS channel to listen
    # @param frequency the frequency to expect messages (used to print
    #           warning statements to console)
    # @param message_extractor function to preprocess the message into
    #           a desired format
    # @param queue_size ROS subscriber queue (None = infinite)
    def __init__(self, node_name, message_type, listen_channel,
                 frequency, message_extractor=None, queue_size=None):
        #Last message returned to callers from this class
        self.last_msg_returned   = None   
        #Local time of last received message
        self.last_call_back      = None   
        self.delay_tolerance     = 1/frequency #in seconds
        self.reading             = {'message':None, 'msg_id':-1}
        self.curid               = 0
        self.message_extractor = message_extractor

        def callback(*msg):
            #If this is a tuple (using message filter)
            if 'header' in dir(msg):
                if msg.__class__ == ().__class__:
                    msg_number = msg[0].header.seq
                else:
                    msg_number = msg.header.seq
            else:
                msg_number = self.curid
                self.curid += 1

            #*msg makes everything a tuple.  If length is one, msg = (msg, )
            if len(msg) == 1:
                msg = msg[0]
            
            self.reading  = {'message':msg, 'msg_id':msg_number}

            #Check for delayed messages
            #record when we have been called back last
            self.last_call_back = time.time() 

        if message_type.__class__ == [].__class__:
            import message_filters
            subscribers = [message_filters.Subscriber(channel, mtype)\
                            for channel, mtype in \
                            zip(listen_channel, message_type)]
            queue_size = 10
            ts = message_filters.TimeSynchronizer(subscribers, queue_size)
            ts.registerCallback(callback)
        else:
            rospy.Subscriber(listen_channel, message_type, callback,
                             queue_size = queue_size)

        self.node_name = node_name
        #print node_name,': subscribed to', listen_channel
        rospy.loginfo('%s: subscribed to %s' % (node_name, listen_channel))

    def _check_for_delivery_hiccups(self):
        #If have received a message in the past
        if self.last_call_back != None:
            #Calculate how it has been
            time_diff = time.time() - self.last_call_back
            #If it has been longer than expected hz, complain
            if time_diff > self.delay_tolerance:
                print self.node_name, \
                        ': have not heard back from publisher in', \
                        time_diff, 's'

    def _wait_for_first_read(self, quiet=False):
        if not quiet:
            rospy.loginfo('%s: waiting for reading ...' % self.node_name)
        while self.reading['message'] == None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    ## Supported use cases
    # rfid   - want to get a reading, can be stale, 
    #           no duplication allowed (allow None), 
    #           query speed important
    # hokuyo - want to get a reading, can be stale, 
    #           no duplication allowed (don't want a None), 
    #           willing to wait for new data (default)
    # ft     - want to get a reading, can be stale, 
    #           duplication allowed    (don't want a None), 
    #           query speed important
    # NOT ALLOWED   duplication allowed, willing to wait for new data
    def read(self, allow_duplication=False, willing_to_wait=True, 
            warn=False, quiet=True):
        if allow_duplication:
            if willing_to_wait:
                raise RuntimeError('Invalid settings for read.')
            else: 
                # ft - want to get a reading, can be stale,
                # duplication allowed (but don't want a None), query
                # speed important
                #self._wait_for_first_read(quiet)
                reading                = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
        else:
            if willing_to_wait:
                # hokuyo - want to get a reading, can be stale, no
                # duplication allowed (don't want a None), willing to
                # wait for new data (default)
                self._wait_for_first_read(quiet)
                while self.reading['msg_id'] == self.last_msg_returned \
                        and not rospy.is_shutdown():
                    if warn:
                        self._check_for_delivery_hiccups()
                    rospy.sleep(1/1000.0)
                reading = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
            else:
                # rfid   - want to get a reading, can be stale, no
                # duplication allowed (allow None),        query speed
                # important
                if self.last_msg_returned == self.reading['msg_id']:
                    return None
                else:
                    reading = self.reading
                    self.last_msg_returned = reading['msg_id']
                    if self.message_extractor is not None:
                        return self.message_extractor(reading['message'])
                    else:
                        return reading['message']

## Set and get the current position of the PR2 joints
class Joint:

    ## Constructor
    # @param name name of joint group (/NAME/joints).
    # @param joint_provider A GenericListener that listens to joint_states.
    def __init__(self, name, joint_provider):
        self.joint_provider = joint_provider
        try:
            self.joint_names = rospy.get_param('/%s/joints' % name)
        except KeyError, e:
            self.joint_names = ['l_shoulder_pan_joint', 
                    'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 
                    'l_elbow_flex_joint', 'l_forearm_roll_joint', 
                    'l_wrist_flex_joint', 'l_wrist_roll_joint']

        self.pub = rospy.Publisher('%s/command' % name, tm.JointTrajectory)
        self.names_index = None
        self.zeros = [0 for j in range(len(self.joint_names))]

    ## Gets the current pose
    # @param joint_states if given a joint states message, extracts
    # the current pose from the message instead of querying robot
    # (leave this blank normally).
    # @return a nx1 matrix of joint angles
    def pose(self, joint_states=None):
        if joint_states == None:
            joint_states = self.joint_provider()

        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(joint_states.name):
                self.names_index[n] = i
            self.joint_idx = [self.names_index[n] for n in self.joint_names]

        return (np.matrix(joint_states.position).T)[self.joint_idx, 0]

    ## Creates a JointTrajectory message
    # @param pos_mat nxm np.matrix of joint angles, where each entry is a
    # keyframe (n is number of keyframes, m is the number of joints)
    # @param times the times alloted to each key frame
    # @param vel_mat nxm np.matrix of velocities in
    # JointTrajectoryPoint.
    def _create_trajectory(self, pos_mat, times, vel_mat=None):
        #Make JointTrajectoryPoints
        points = [tm.JointTrajectoryPoint() for i in range(pos_mat.shape[1])]
        for i in range(pos_mat.shape[1]):
            points[i].positions = pos_mat[:,i].A1.tolist()
            points[i].accelerations = self.zeros
            if vel_mat == None:
                points[i].velocities = self.zeros
            else:
                points[i].velocities = vel_mat[:,i].A1.tolist()

        for i in range(pos_mat.shape[1]):
            points[i].time_from_start = rospy.Duration(times[i])

        #Create JointTrajectory
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points = points
        jt.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        return jt

    ## Tells the robot to go to a set of joint poses
    # @param pos_mat nxm np.matrix of joint angles, where each entry is a
    # keyframe (n is number of keyframes, m is the number of joints)
    # @param times the times alloted to each key frame
    def set_poses(self, pos_mat, times):
        joint_trajectory = self._create_trajectory(pos_mat, times)
        print 'Sending message', joint_trajectory
        self.pub.publish(joint_trajectory)

    ## Gets names of the joints in this joint group
    def get_joint_names(self):
        return self.joint_names


## Interfaces with the PR2's Arm
class PR2Arm(Joint):

    ## Constructor
    # @param joint_provider A GenericListener that listens to joint_states.
    # @param tf_listener TFListener class
    # @param arm either 'l' or 'r'
    def __init__(self, joint_provider, tf_listener, arm):
        joint_controller_name = arm + '_arm_controller'
        cart_controller_name = arm + '_cart'
        Joint.__init__(self, joint_controller_name, joint_provider)
        self.arm = arm
        self.tf_listener = tf_listener
        self.client = actionlib.SimpleActionClient(\
                '/%s/joint_trajectory_action' % joint_controller_name, 
                pm.JointTrajectoryAction)
        self.joint_controller_name = joint_controller_name

        self.cart_posure_pub = rospy.Publisher("/%s/command_posture" \
                % cart_controller_name, stdm.Float64MultiArray).publish
        self.cart_pose_pub = rospy.Publisher("/%s/command_pose" \
                % cart_controller_name, gm.PoseStamped).publish
        if arm == 'l':
            self.full_arm_name = 'left'
        else:
            self.full_arm_name = 'right'

        self.limits_dict, self.vel_limit_dict = self._limits()
        self.POSTURES = {
            'off':       np.matrix([]),
            'mantis':    np.matrix([0, 1, 0,  -1, 3.14, -1, 3.14]).T,
            'elbowupr':  np.matrix([-0.79,0,-1.6,  9999, 9999, 9999, 9999]).T,
            'elbowupl':     np.matrix([0.79,0,1.6 , 9999, 9999, 9999, 9999]).T,
            'old_elbowupr': np.matrix([-0.79,0,-1.6, -0.79,3.14, -0.79,5.49]).T,
            'old_elbowupl': np.matrix([0.79,0,1.6, -0.79,3.14, -0.79,5.49]).T,
            'elbowdownr':   np.matrix([-0.028262077316910873, 
                1.2946342642324222, -0.25785640577652386, 
                -1.5498884526859626]).T, 
            'elbowdownl':   np.matrix([-0.0088195719039858515, 
                1.2834828245284853, 0.20338442004843196, 
                -1.5565279256852611]).T
            }

    ## Get joint limits
    # @return a dictionary of limits of the form
    #       {joint name: [min position, max position], ...}
    def get_limits(self):
        return self.limits_dict

    ## Get velocity limits
    # @return a dictionary of the form {joint name: max velocity, ...}
    def get_vel_limits(self):
        return self.vel_limit_dict

    ## Calls ros service to get joint limits
    def _limits(self):
        service_name = '/pr2_%s_arm_kinematics/get_ik_solver_info' \
                % self.full_arm_name
        proxy = rospy.ServiceProxy(service_name, 
                GetKinematicSolverInfo, persistent=True)
        rospy.loginfo('Waiting for %s' % service_name)
        rospy.wait_for_service(service_name)
        info = proxy().kinematic_solver_info
        limit_dict = {}
        vel_limit_dict = {}
        for idx, name in enumerate(info.joint_names):
            limit = info.limits[idx]
            if limit.has_position_limits:
                limit_dict[name] = [limit.min_position, limit.max_position]
            if limit.has_velocity_limits:
                vel_limit_dict[name] = limit.max_velocity
        return limit_dict, vel_limit_dict

    ## Sets the posture on the Cartesian controller
    def set_posture(self, posture_mat):
        self.cart_posure_pub(\
                stdm.Float64MultiArray(data=posture_mat.A1.tolist()))

    ## Send a cartesian pose to *_cart controllers
    # @param trans len 3 list
    # @param rot len 3 list
    # @param frame string
    # @param msg_time float
    def set_cart_pose(self, trans, rot, frame, msg_time):
        ps = gm.PoseStamped()
        for i, field in enumerate(['x', 'y', 'z']):
            exec('ps.pose.position.%s = trans[%d]' % (field, i))
        for i, field in enumerate(['x', 'y', 'z', 'w']):
            exec('ps.pose.orientation.%s = rot[%d]' % (field, i))
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time(msg_time)
        self.cart_pose_pub(ps)

    ## Gives the robot a series of keyframes to execute with associated times.
    # @param pos_mat numpy column matrix of poses (each column is a keyframe)
    # @param times numpy array of times
    # @param vel_mat numpy velocity constraint at each of the keyframe.
    # @param block should this function block (boolean)
    # @param allow_spins should we allow the forearm or wrist to spin (boolean)
    def set_poses(self, pos_mat, times, vel_mat=None, block=True, 
            allow_spins=False):

        cur_pose = self.pose()
        pos_mat[4,:] = np.matrix(angle_consistency_check(\
                cur_pose[4,0], pos_mat[4,:].A1, allow_spins))
        pos_mat[6,:] = np.matrix(angle_consistency_check(\
                cur_pose[6,0], pos_mat[6,:].A1, allow_spins))

        pos_mat = np.column_stack([cur_pose, pos_mat])
        times   = np.concatenate(([0], times))
        times = times + .1
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    ## Like set_poses but send a single pose.
    # @param pos pose to go to, nx1 matrix
    # @param nsecs number of seconds that this trajectory should take
    # @param block whether to block until finish
    def set_pose(self, pos, nsecs=5., block=True):
        for i in range(2):
            cpos = self.pose()
        pos[4,0] = unwrap(cpos[4,0], pos[4,0])
        pos[6,0] = unwrap(cpos[6,0], pos[6,0])
        self.set_poses(np.column_stack([pos]), np.array([nsecs]), block=block)

    ## Stops the currently executing joint trajectory
    def stop_trajectory_execution(self):
        self.client.cancel_all_goals()

    ## Checks whether there are active goals
    # @return boolean
    def has_active_goal(self):
        s = self.client.get_state()
        if s == amsg.GoalStatus.ACTIVE or s == amsg.GoalStatus.PENDING:
            return True
        else:
            return False

    def set_poses_monitored(self, pos_mat, times, vel_mat=None, block=True, 
            time_look_ahead=.050):
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()


    ## Gets pose of the gripper in Cartesian coordinates
    # @param frame the frame to report the returned pose in
    # @return 4x4 transform matrix
    def pose_cartesian(self, frame='base_link'):
        gripper_tool_frame = self.arm + '_gripper_tool_frame'
        return tfu.transform(frame, gripper_tool_frame, self.tf_listener)

    ## Gets pose of the gripper in Cartesian coordinates in tf format
    # @return 3x1 position matrix, 4x1 quaternion matrix
    def pose_cartesian_tf(self, frame='base_link'):
        p, r = tfu.matrix_as_tf(self.pose_cartesian(frame))
        return np.matrix(p).T, np.matrix(r).T


## Interfaces with the torso/spin controllers
class PR2Torso(Joint):

    ## Constructor
    # @param joint_provider a GenericListener object that listens to
    # the joint states topic
    def __init__(self, joint_provider):
        Joint.__init__(self, 'torso_controller', joint_provider)
        self.torso = actionlib.SimpleActionClient(
                'torso_controller/position_joint_action', 
                pm.SingleJointPositionAction)
        rospy.loginfo('waiting for torso_controller/position_joint_action')

    
    ## Set the pose of the torso/spine
    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()

## Interfaces with head pan/tilt
class PR2Head(Joint):

    ## Constructor
    # @param joint_provider GenericListener class that listens to
    # joint_states.
    def __init__(self, joint_provider):
        Joint.__init__(self, 'head_traj_controller', joint_provider)
        self.head_client = actionlib.SimpleActionClient(
                'head_traj_controller/point_head_action', 
                pm.PointHeadAction)

    ## Set head pose
    # @param pos 2x1 numpy.matrix
    # @param length how long should the head take to get to this pose (double)
    # @param block whether to block (bool)
    def set_pose(self, pos, length=5., block=False):
        for i in range(2):
            cpos = self.pose()
        MIN_TIME = .1
        self.set_poses(np.column_stack([cpos, pos]), 
                np.array([MIN_TIME, MIN_TIME+length]))

    ## Points the head at a given 3D point (through point_head_action)
    # @param pt3d 3x1 numpy.matrix of a 3D point
    # @param frame Frame that the point is in (string)
    # @param pointing_frame Pointing frame field of point_head_action
    # @param pointing_axis Pointing axis field of point_head_action
    # @param wait Whether to block until motion completes before returning.
    def look_at(self, pt3d, frame='base_link', 
                pointing_frame="wide_stereo_link",
                pointing_axis=np.matrix([1, 0, 0.]).T, wait=True):
        g = pm.PointHeadGoal()
        g.target.header.frame_id = frame
        g.target.point = gm.Point(*pt3d.T.A1.tolist())

        g.pointing_frame = pointing_frame
        g.pointing_axis.x = pointing_axis[0,0]
        g.pointing_axis.y = pointing_axis[1,0]
        g.pointing_axis.z = pointing_axis[2,0]
        g.min_duration = rospy.Duration(1.0)
        g.max_velocity = 10.

        self.head_client.send_goal(g)
        if wait:
            self.head_client.wait_for_result(rospy.Duration(1.))
        if self.head_client.get_state() == amsg.GoalStatus.SUCCEEDED:
            return True
        else:
            return False

## Interfaces with the PR2's base
class PR2Base:

    ## Constructor
    # @param tflistener TF Listener
    def __init__(self, tflistener):
        self.tflistener = tflistener
        self.client = actionlib.SimpleActionClient('move_base', 
                mm.MoveBaseAction)
        self.go_angle_client = actionlib.SimpleActionClient('go_angle', 
                hm.GoAngleAction)
        self.go_xy_client = actionlib.SimpleActionClient('go_xy', 
                hm.GoXYAction)

    ## Sends a MoveBaseGoal
    # @param t Translation (2 elements 1D array, or list)
    # @param r Rotation (4 elements 1D array, or list)
    # @param frame Frame that goal is in.
    # @param block Whether to block until motion completes.
    def set_pose(self, t, r, frame, block=True):
        g = mm.MoveBaseGoal()
        p = g.target_pose
        
        p.header.frame_id = frame
        p.header.stamp = rospy.get_rostime()
        p.pose.position.x = t[0]
        p.pose.position.y = t[1]
        p.pose.position.z = 0
        
        p.pose.orientation.x = r[0]
        p.pose.orientation.y = r[1]
        p.pose.orientation.z = r[2]
        p.pose.orientation.w = r[3]
    
        self.client.send_goal(g)
        if block:
            self.client.wait_for_result()
        return self.client.get_state()

    ## Gets the current base pose
    # @return a 4x4 transform matrix.
    def get_pose(self):
        p_base = tfu.transform('map', 'base_footprint', self.tflistener) \
                * tfu.tf_as_matrix(([0., 0., 0., 1.],\
                            tr.quaternion_from_euler(0,0,0)))
        return tfu.matrix_as_tf(p_base)


    ## Turns to given angle using pure odometry
    # @param angle angle to turn to (in odom frame, radians)
    # @param block whether to block until motion completes
    def turn_to(self, angle, block=True):
        goal = hm.GoAngleGoal()
        goal.angle = angle
        self.go_angle_client.send_goal(goal)
        #print 'SENT TURN GOAL'
        if block:
            rospy.loginfo('turn_to: waiting for turn..')
            self.go_angle_client.wait_for_result()
            rospy.loginfo('turn_to: done.')
            

    ## Turns a relative amount given angle using pure odometry
    # @param delta_ang amount to turn by (in radians)
    # @param block whether to block until motion completes
    # @param overturn should we overturn for small motions (to over
    #                   come sticktion)
    def turn_by(self, delta_ang, block=True, overturn=False):
        #overturn
        if overturn and (abs(delta_ang) < math.radians(10.)):
            #turn in that direction by an extra 15 deg
            turn1 = np.sign(delta_ang) * math.radians(15.) + delta_ang
            turn2 = -np.sign(delta_ang) * math.radians(15.)
            rospy.loginfo('Requested really small turn angle.'\
                    +'  Using overturn trick.')
            #pdb.set_trace()
            self._turn_by(turn1, block=True)
            time.sleep(3) #TODO remove this restriction
            self._turn_by(turn2, block)
        else:
            self._turn_by(delta_ang, block)

    ## Helper mehod for turn_by, creates the actual turn_to command
    # @param delta_ang change in angle to turn by
    # @param block whether to block until motion completes
    def _turn_by(self, delta_ang, block=True):
        current_ang_odom = tr.euler_from_matrix(
                tfu.transform('base_footprint',
                    'odom_combined', self.tflistener)[0:3, 0:3], 'sxyz')[2]
        self.turn_to(current_ang_odom + delta_ang, block)


    ## Move to xy_loc_bf
    # @param xy_loc_bf 2x1 numpy matrix representing a 2D position (in
    #                   base_frame)
    # @param block whether to block
    def move_to(self, xy_loc_bf, block=True):
        goal = hm.GoXYGoal()
        goal.x = xy_loc_bf[0,0]
        goal.y = xy_loc_bf[1,0]
        self.go_xy_client.send_goal(goal)
        if block:
            self.go_xy_client.wait_for_result()

## Creates a GenericListener that listens to joint_states
def create_joint_provider():
    jl = GenericListener('joint_state_listener', sm.JointState, 
            'joint_states', 100)
    return ft.partial(jl.read, allow_duplication=False, 
            willing_to_wait=True, warn=False, quiet=True)

## Container PR2 class
class PR2:

    def __init__(self, tf_listener):
        self.tf_listener = tf_listener
        joint_provider = create_joint_provider()
        self.left  = PR2Arm(joint_provider, tf_listener, 'l')
        self.right = PR2Arm(joint_provider, tf_listener, 'r')
        self.torso = PR2Torso(joint_provider)
        self.head  = PR2Head(joint_provider)
        self.base  = PR2Base(tf_listener)
        self.controller_manager = ControllerManager()

if __name__ == '__main__':
    unittest.main()
