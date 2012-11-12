import rcommander.tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pr2_common_action_msgs.msg as ca 
import functools as ft
import pr2_controllers_msgs.msg as pm

## Tool for setting how wide the gripper opens/closes and with how much effort
class GripperTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'gripper', 'Gripper', GripperState)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        #Left or right
        self.radio_boxes, self.radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'gripper_arm')
        #Opening distance
        self.gripper_box = tu.SliderBox(pbox, 0.01, 8.4, 0.02, .01, 'gripper', unit='cm')
        #Effort
        self.effort_box = tu.SliderBox(pbox, 50., 200., 0., 40., 'effort', unit='')

        formlayout.addRow('&Side', self.radio_boxes)
        formlayout.addRow('&Gripper Opening', self.gripper_box.container)
        formlayout.addRow('&Effort', self.effort_box.container)
        pbox.update()
        self.reset()

    ## Inherited
    def new_node(self, name=None):
        selected_arm = None
        for r in self.radio_buttons:
            if r.isChecked():
                selected_arm = str(r.text()).lower()
        if selected_arm == None:
            raise RuntimeError('No arm selected!')

        gsize = self.gripper_box.value()
        effort = self.effort_box.value()

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return GripperState(nname, selected_arm, gsize, effort)
    
    ## Inherited
    def set_node_properties(self, gripper_state):
        if gripper_state.arm == 'left':
            self.radio_buttons[0].setChecked(True)
        if gripper_state.arm == 'right':
            self.radio_buttons[1].setChecked(True)

        self.gripper_box.set_value(gripper_state.gripper_size)
        self.effort_box.set_value(gripper_state.effort)

    ## Inherited
    def reset(self):
        self.gripper_box.set_value(0.0)
        self.effort_box.set_value(50.)
        self.radio_buttons[1].setChecked(True)


class GripperState(tu.SimpleStateBase): 

    ## Constructor
    def __init__(self, name, arm, gripper_size, effort):
        if arm == 'left':
            action = 'l_gripper_controller/gripper_action'
        elif arm == 'right':
            action = 'r_gripper_controller/gripper_action'

        tu.SimpleStateBase.__init__(self, name, \
                action, pm.Pr2GripperCommandAction,
                goal_cb_str = 'ros_goal')

        self.gripper_size = gripper_size
        self.effort = effort
        self.arm = arm

    ## Inherited
    def ros_goal(self, userdata, default_goal):
        return pm.Pr2GripperCommandGoal(pm.Pr2GripperCommand(
            position=self.gripper_size/100., max_effort=self.effort))

