#import roslib; roslib.load_manifest('rcommander_pr2')
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
#import time
#import pr2_common_action_msgs.msg as ca 
import rcommander_pr2_gui.msg as rm
import rospy

class TuckTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'tuck', 'Tuck', TuckState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        #self.speedline = QLineEdit(pbox)

        self.tuck_left = QComboBox(pbox)
        self.tuck_left.addItem('True')
        self.tuck_left.addItem('False')

        self.tuck_right = QComboBox(pbox)
        self.tuck_right.addItem('True')
        self.tuck_right.addItem('False')

        formlayout.addRow("&Left Arm", self.tuck_left)
        formlayout.addRow("&Right Arm", self.tuck_right)
        pbox.update()

    def new_node(self, name=None):
        left = ('True' == str(self.tuck_left.currentText()))
        right = ('True' == str(self.tuck_right.currentText()))
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return TuckState(nname, left, right)
    
    def set_node_properties(self, node):
        self.tuck_left.setCurrentIndex(self.tuck_left.findText(str(node.tuck_left)))
        self.tuck_right.setCurrentIndex(self.tuck_right.findText(str(node.tuck_right)))

    def reset(self):
        self.tuck_left.setCurrentIndex(self.tuck_left.findText('True'))
        self.tuck_right.setCurrentIndex(self.tuck_right.findText('True'))


class TuckState(tu.SimpleStateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, tuck_left, tuck_right):
        tu.SimpleStateBase.__init__(self, name, \
                'rcommander_tuckarms', rm.RCTuckArmsAction,
                goal_cb_str = 'ros_goal') 

        self.tuck_left = tuck_left
        self.tuck_right = tuck_right

    def ros_goal(self, userdata, default_goal):
        goal = rm.RCTuckArmsGoal()
        goal.tuck_left = self.tuck_left
        goal.tuck_right = self.tuck_right
        rospy.loginfo('TuckState: left %s right %s'% (str(goal.tuck_left), str(goal.tuck_right)))
        return goal

    #def get_smach_state(self):
    #
    #def __getstate__(self):
    #    state = tu.SimpleStateBase.__getstate__(self)
    #    my_state = [self.tuck_left, self.tuck_right]
    #    return {'simple_state': state, 'self': my_state}

    #def __setstate__(self, state):
    #    tu.SimpleStateBase.__setstate__(self, state['simple_state'])
    #    self.tuck_left, self.tuck_right = state['self']
