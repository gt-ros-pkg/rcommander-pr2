#import roslib; roslib.load_manifest('rcommander_pr2')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import numpy as np
import actionlib_msgs.msg as am
import smach
import functools as ft
import pypr2.pr2_utils as pu
from rcommander_pr2_gui.srv import MinTime
import trajectory_msgs.msg as tm
import os.path as pt
import roslib
import pypr2.pr2_utils as p2u

def split_joints_list(jlist):
    left = []
    right = []
    for d in jlist:
        if d['arm'] == 'left':
            left.append(d)
        else:
            right.append(d)
    return left, right


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

        jang_list = self.get_joint_angs_list()
        left_list, right_list = split_joints_list(jang_list)

        #arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        arm = self.get_arm_radio()
        if arm == 'left':
            #idx = 0
            pref = 'l_'
            left_arm = True
            ang_list = left_list
        else:
            #idx = 1
            pref = 'r_'
            left_arm = False
            ang_list = right_list

        if len(ang_list) == 0:
            return

        if self.list_manager.get_selected_idx() == None:
            ref = ang_list[-1]
        else:
            sidx = self.list_manager.get_selected_idx()
            ang_list_idx = None
            for idx, d in enumerate(ang_list):
                if d == jang_list[sidx]:
                    ang_list_idx = idx
            if ang_list_idx == None:
                return
            pidx = ang_list_idx - 1
            if pidx < 0:
                return 
            else:
                ref = ang_list[pidx]

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
        return {'arm': self.get_arm_radio(),
                'time': self.time_box.value(), 
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
        self.set_arm_radio(data['arm'])

        if self.element_will_be_added:
            self.element_will_be_added = False

    def live_update_cb(self):
        pos_mat = self.get_robot_joint_angles()
        pos_mat[4,:] = pos_mat[4,:] % (2.*np.pi)
        pos_mat[6,:] = pos_mat[6,:] % (2.*np.pi)

        self.reset_live_update = False
        self.list_manager.display_record({'arm': self.get_arm_radio(),
                                          'time': self.time_box.value(),
                                          'angs': pos_mat.A1.tolist()})
        self.reset_live_update = True

    def live_update_toggle_cb(self, state):
        if not state:
            self.reset_live_update = False
            selected_name = self.list_manager.get_selected_name()
            if selected_name != None:
                self.list_manager.set_selected_by_name(selected_name)
            self.reset_live_update = True

    def add_element_cb(self):
        self.element_will_be_added = True

    def arm_radio_toggled(self, state):
        self.check_all_joint_limits()
        self._check_time_validity(self.time_box.value())

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        items_to_monitor = []

        fields, arm_radio_boxes, buttons = self.make_joint_boxes(pbox, self.rcommander)
        arm_radio_buttons = self.get_arm_radio_buttons()
        for b in arm_radio_buttons:
            self.rcommander.connect(b, SIGNAL('toggled(bool)'), self.arm_radio_toggled)

        formlayout.addRow('&Arm', arm_radio_boxes)
        for field in fields:
            formlayout.addRow(field['name'], field['item'])
            vchanged_func = ft.partial(self._value_changed_validate, joint=field['joint'])
            self.rcommander.connect(field['item'], SIGNAL('valueChanged(double)'), vchanged_func)
            items_to_monitor.append(field['item'])

        self.time_box = tu.double_spin_box(pbox, 0, 1000, .2)
        self.time_box.setValue(3.)
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
        sstate = JointSequenceState(nname, self.list_manager.get_data())
        return sstate

    def set_node_properties(self, my_node):
        self.list_manager.set_data(my_node.joint_waypoints)
        self.list_manager.select_default_item()
        #self.set_arm_radio(my_node.arm)

    def reset(self):
        self.set_arm_radio('right')
        self.set_all_fields_to_zero()
        self.pose_button.setEnabled(True)
        self.list_manager.reset()
        self.stop_timer()


class JointSequenceState(tu.StateBase): 

    def __init__(self, name, joint_waypoints):
        tu.StateBase.__init__(self, name)
        self.joint_waypoints = joint_waypoints

    def get_smach_state(self):
        return JointSequenceStateSmach(self.joint_waypoints)


class JointSequenceStateSmach(smach.State): 

    TIME_OUT_FACTOR = 3.

    def __init__(self, joint_waypoints):
        smach.State.__init__(self, outcomes = ['aborted', 'succeeded', 'preempted', 'timed_out'], 
                             input_keys = [], output_keys = [])
        self.joint_waypoints = joint_waypoints
        self.l_arm_obj = None
        self.r_arm_obj = None


    def set_robot(self, pr2):
        if pr2 == None:
            return
        self.l_arm_obj = pr2.left
        self.r_arm_obj = pr2.right
        self.controller_manager = pr2.controller_manager


    def execute(self, userdata):
        left_points, right_points = split_joints_list([wp['data'] for wp in self.joint_waypoints])
        if len(left_points) > 0 and len(right_points) > 0:
            arms = 'both'
        elif len(left_points) > 0:
            arms = 'left'
        elif len(right_points) > 0:
            arms = 'right'
            
        status, started, stopped = self.controller_manager.joint_mode(arms)
        clients = []
        states = []
        trajectory_time_out = -1
        for arm_obj, points in zip([self.l_arm_obj, self.r_arm_obj], [left_points, right_points]):
            if len(points) == 0:
                continue
            #Construct trajectory command
            times = []
            wps = []
            for d in points:
                wps.append(np.matrix(d['angs']).T)
                times.append(d['time'])

            arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
            clients.append(arm_obj.client)
            states.append(arm_obj.client.get_state())
            time_out = JointSequenceStateSmach.TIME_OUT_FACTOR * np.sum(times)
            trajectory_time_out = max(time_out, trajectory_time_out)

        # returns one of failed, preempted, timed_out, or succeeded
        result = tu.monitor_goals(self, clients, 'JointSequenceState', trajectory_time_out)
        self.controller_manager.switch(stopped, started)
        if result == 'failed':
            return 'aborted'
        return result








































        #succeeded = False
        #preempted = False
        #r = rospy.Rate(30)
        #start_time = rospy.get_time()
        #while not rospy.is_shutdown():
        #    #we have been preempted
        #    if self.preempt_requested():
        #        rospy.loginfo('JointSequenceState: preempt requested')
        #        [c.cancel_goal() for c in clients]
        #        self.service_preempt()
        #        preempted = True
        #        break

        #    if (rospy.get_time() - start_time) > trajectory_time_out:
        #        [c.cancel_goal() for c in clients]
        #        rospy.loginfo('JointSequenceState: timed out!')
        #        succeeded = False
        #        break

        #    #print tu.goal_status_to_string(state)
        #    stopped_states = [s not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING] for s in states]
        #    if np.all(stopped_states):
        #        #succeed if all states succeeded
        #        if np.all([s == am.GoalStatus.SUCCEEDED for s in states]):
        #            rospy.loginfo('JointSequenceState: Succeeded!')
        #            succeeded = True
        #        break
        #        #failed if any state failed

        #    states = [c.get_state() for c in clients]
        #    r.sleep()


        #if preempted:
        #    return 'preempted'

        #if succeeded:
        #    return 'succeeded'

        #if state == am.GoalStatus.ABORTED:
        #    return 'aborted'

        #return 'failed'

