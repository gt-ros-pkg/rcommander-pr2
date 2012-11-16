#import roslib; roslib.load_manifest('rcommander_pr2')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import pr2_controllers_msgs.msg as pm

## Tool to produce motions that move the spine.
class SpineTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_spine', 'Spine', SpineState)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.spine_box = tu.SliderBox(pbox, 15., 29.5, 1., .05, 'spine', unit='cm')
        formlayout.addRow('&Height', self.spine_box.container)
        pbox.update()

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return SpineState(nname, self.spine_box.value()/100.)

    ## Inherited
    def set_node_properties(self, my_node):
        self.spine_box.set_value(my_node.position*100.)

    ## Inherited
    def reset(self):
        self.spine_box.set_value(15.)

class SpineState(tu.SimpleStateBase): 

    ## Constructor
    # @param name of state
    # @param position position of spine (float)
    def __init__(self, name, position):
        tu.SimpleStateBase.__init__(self, name, \
                'torso_controller/position_joint_action', 
                pm.SingleJointPositionAction, 
                goal_cb_str = 'ros_goal')
        self.position = position

    ## Inherited
    def ros_goal(self, userdata, default_goal):
        return pm.SingleJointPositionGoal(position = self.position)

