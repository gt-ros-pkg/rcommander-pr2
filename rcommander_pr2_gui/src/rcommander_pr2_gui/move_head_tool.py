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

## Sends the head to a series of pan tilt angles
# TODO: refactor to use new LiveUpdateTool class
class MoveHeadTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_head_ang', 'Move Head (ang)', MoveHeadState)
        self.head_angs_list = None
        self.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.status_bar_timer = QTimer()
        self.rcommander.connect(self.status_bar_timer, SIGNAL('timeout()'), self.live_update_cb)
        self.live_update = False
        self.current_update_color = p2u.create_color(0,0,0,255)

    ## Call back for live update
    def live_update_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.list_manager.display_record({'angs': poses.A1.tolist(), 'time': self.time_box.value()})

    ## Get the current head position
    def current_pose_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.joint_boxes[0].setValue(np.degrees(poses[0,0]))
        self.joint_boxes[1].setValue(np.degrees(poses[1,0]))

    ## Callback for 'Live Update' button, sets mode of dialog box depending on
    # state of button.
    def update_selected_cb(self):
        if self.live_update_button.text() == 'Live Update':
            self.set_update_mode(True)
        else:
            self.set_update_mode(False)

    ## Callback for ListManager to get data from GUI fields
    def get_current_data_cb(self):
        head_angs = []
        for name in self.joint_names:
            exec('obj = self.%s' % name)
            head_angs.append(np.radians(obj.value()))
        return {'angs': head_angs,
                'time': self.time_box.value()}

    ## Callback for ListManager to set data to GUI fields
    def set_current_data_cb(self, data):
        for idx, name in enumerate(self.joint_names):
            exec('obj = self.%s' % name)
            obj.setValue(np.degrees(data['angs'][idx]))
        self.time_box.setValue(data['time'])

    ## Inherited
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

    ## Inherited
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

    ## Inherited
    def reset(self):
        for box in self.joint_boxes:
            box.setValue(0)
        self.time_box.setValue(1.)
        self.list_manager.reset()
        self.list_manager.set_default_selection()

    ## Inherited
    def set_node_properties(self, my_node):
        self.list_manager.set_data(my_node.joint_waypoints)
        self.list_manager.select_default_item()

    ## Set state of property dialog, colors text in text fields to show that
    # things are being updated live!
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

        if not on:
            self.live_update_toggle_cb(self.live_update)

    ## Manages interaction between list manager and the live update callbacks
    # (internal method).
    def live_update_toggle_cb(self, state):
        if not state:
            self.reset_live_update = False
            selected_name = self.list_manager.get_selected_name()
            if selected_name != None:
                self.list_manager.set_selected_by_name(selected_name)
            self.reset_live_update = True

class MoveHeadState(tu.StateBase):

    ## Constructor
    def __init__(self, name, joint_waypoints):
        tu.StateBase.__init__(self, name)
        self.joint_waypoints = joint_waypoints

    ## Inherited
    def get_smach_state(self):
        return MoveHeadStateSmach(self.joint_waypoints)


class MoveHeadStateSmach(smach.State):

    ## Multiple of trajectory length that controls how long before we timeout
    # movement.
    TIME_OUT_FACTOR = 3.

    ## Constructor
    def __init__(self, joint_waypoints):
        smach.State.__init__(self, outcomes=['preempted', 'done'], 
			     input_keys=[], output_keys=[])
        self.joint_waypoints = joint_waypoints

    ## Inherited
    def set_robot(self, robot):
        if robot != None:
            self.head_obj = robot.head
            self.controller_manager = robot.controller_manager

    ## Inherited
    def execute(self, userdata):
        #Construct trajectory command
        times = []
        wps = []
        for x in self.joint_waypoints:
            wps.append(np.matrix(x['data']['angs']).T)
            times.append(x['data']['time'])

        self.head_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)))

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

