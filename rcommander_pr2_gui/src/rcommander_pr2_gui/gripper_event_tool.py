import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pr2_gripper_sensor_msgs.msg as gr
import actionlib_msgs.msg as am
import actionlib
import smach
import rcommander.tool_utils as tu
import rcommander.graph_model as gm
import rcommander.sm_thread_runner as smtr

## Calls the gripper event detector
class GripperEventTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'gripper_event', 'Gripper Event', GripperEventState)
        self.child_gm = None

    ## Inherited (optional function of ToolBase)
    def set_child_node(self, child_node):
        self.child_gm = gm.GraphModel()
        self.child_gm.add_node(child_node)
        self.child_gm.set_start_state(child_node.get_name())
        self.child_gm.set_document(gm.FSMDocument(child_node.get_name(), modified=False, real_filename=False))

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.gripper_radio_boxes, self.gripper_radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'gripper_event_arm')
        self.event_box = QComboBox(pbox)
        for event in GripperEventStateSmach.EVENT_LIST:
            self.event_box.addItem(event)
        self.accel_box = tu.SliderBox(pbox, 8.25, 30, 0., 1, 'gripper_accel', unit='m/s^2')
        self.slip_box = tu.SliderBox(pbox, .01, -.5, .5, .1, 'gripper_slip', unit='')

        formlayout.addRow('&Gripper', self.gripper_radio_boxes)
        formlayout.addRow('&Event', self.event_box)
        formlayout.addRow('&Acceleration', self.accel_box.container)
        formlayout.addRow('&Slip', self.slip_box.container)
        pbox.update()
        self.reset()

    ## Inherited
    def new_node(self, name=None):
        #print 'gripper event new node called'
        if self.child_gm == None:
            raise RuntimeError('No child node!')
        selected_arm = None
        for r in self.gripper_radio_buttons:
            if r.isChecked():
                selected_arm = str(r.text()).lower()
        if selected_arm == None:
            raise RuntimeError('No arm selected!')

        event_type = str(self.event_box.currentText())
        accel_val = self.accel_box.value()
        slip_val = self.slip_box.value()

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return GripperEventState(nname, self.child_gm, selected_arm, event_type, accel_val, slip_val)
    
    ## Inherited
    def set_node_properties(self, gripper_event_state):
        if gripper_event_state.arm == 'left':
            self.gripper_radio_buttons[0].setChecked(True)
        if gripper_event_state.arm == 'right':
            self.gripper_radio_buttons[1].setChecked(True)

        self.event_box.setCurrentIndex(self.event_box.findText(gripper_event_state.event_type))
        self.accel_box.set_value(gripper_event_state.accel)
        self.slip_box.set_value(gripper_event_state.slip)
        self.child_gm = gripper_event_state.child_gm

    ## Inherited
    def reset(self):
        self.gripper_radio_buttons[1].setChecked(True)
        self.event_box.setCurrentIndex(self.event_box.findText(GripperEventStateSmach.EVENT_LIST[0]))
        self.accel_box.set_value(8.25)
        self.slip_box.set_value(.01)
        self.child_gm = None

class GripperEventStateSmach(smach.State): 

    EVENT_LIST = ['accel', 'slip', 'finger side or accel', 'slip and accel', 'finger side, slip or accel']

    EVENT_CODES = {'accel':                      gr.PR2GripperEventDetectorCommand.ACC,
                   'slip':                       gr.PR2GripperEventDetectorCommand.SLIP,
                   'finger side or accel':       gr.PR2GripperEventDetectorCommand.FINGER_SIDE_IMPACT_OR_ACC,
                   'slip and accel':             gr.PR2GripperEventDetectorCommand.SLIP_AND_ACC,
                   'finger side, slip or accel': gr.PR2GripperEventDetectorCommand.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC}

    EVENT_OUTCOME = 'detected_event'

    ## Constructor
    def __init__(self, child_gm, arm, event_type, accel, slip):
        self.child_gm = child_gm 
        #Setup our action server
        if arm == 'left':
            a = 'l'
        elif arm == 'right':
            a = 'r'
        else:
            raise RuntimeError('Error')
        evd_name = a + '_gripper_sensor_controller/event_detector'
        self.action_client = actionlib.SimpleActionClient(evd_name, gr.PR2GripperEventDetectorAction)
        self.event_type = event_type
        self.accel = accel
        self.slip = slip

    ## Inherited
    def set_robot(self, robot):
        self.robot = robot
        input_keys = []
        output_keys = []
        outcomes = []
        if self.child_gm != None:
            sm = self.child_gm.create_state_machine(robot)
            input_keys = list(sm.get_registered_input_keys())
            output_keys = list(sm.get_registered_output_keys())
            outcomes = list(sm.get_registered_outcomes()) + [GripperEventStateSmach.EVENT_OUTCOME, 'failed']
        smach.State.__init__(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys)

    def _detected_event(self):
        state = self.action_client.get_state()
        gripper_event_detected = state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]
        return gripper_event_detected

    ## Inherited
    def execute(self, userdata):
        goal = gr.PR2GripperEventDetectorGoal()
        goal.command.acceleration_trigger_magnitude = self.accel
        goal.command.slip_trigger_magnitude = self.slip
        goal.command.trigger_conditions = GripperEventStateSmach.EVENT_CODES[self.event_type]
        self.action_client.send_goal(goal)

        sm = self.child_gm.create_state_machine(self.robot, userdata=userdata._ud)
        self.child_gm.run(self.child_gm.get_start_state(), state_machine=sm)
        rthread = self.child_gm.sm_thread['run_sm']
        
        event = self._detected_event()
        preempted = False
        r = rospy.Rate(100)
        while not event and not rospy.is_shutdown():
            if rthread.exception != None:
                raise rthread.exception

            if rthread.outcome != None:
                rospy.loginfo('Gripper Event Tool: child node finished with outcome ' + rthread.outcome)
                break

            if not rthread.isAlive():
                rospy.loginfo('Gripper Event Tool: child node died')
                return 'failed'

            if self.preempt_requested():
                rospy.loginfo('Gripper Event Tool: preempt requested')
                rthread.preempt()
                self.service_preempt()
                preempted = True
                break

            event = self._detected_event() 
            if event:
                rospy.loginfo('Gripper Event Tool: DETECTED EVENT')
                rthread.preempt()

            r.sleep()

        if preempted:
            return 'preempted'

        if event:
            return self.EVENT_OUTCOME
        else:
            return rthread.outcome


class GripperEventState(tu.EmbeddableState):

    def __init__(self, name, child_gm, arm, event_type, accel, slip):
        tu.EmbeddableState.__init__(self, name, child_gm)
        self.arm = arm
        self.event_type = event_type
        self.accel = accel
        self.slip = slip

    ## Inherited
    def get_smach_state(self):
        return GripperEventStateSmach(self.get_child(),
                self.arm, self.event_type, self.accel, self.slip)

    ## Inherited
    def recreate(self, new_graph_model):
        return GripperEventState(self.get_name(), new_graph_model, 
                self.arm, self.event_type, self.accel, self.slip)


