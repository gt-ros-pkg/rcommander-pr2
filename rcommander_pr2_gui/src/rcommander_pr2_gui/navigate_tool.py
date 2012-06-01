#import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import move_base_msgs.msg as mm
import math


#
# controller and view
# create and edits smach states
class NavigateTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate', 'Navigate (planner)', NavigateState)
        self.tf_listener = rcommander.tf_listener
        self.default_frame = 'map'
        self.robot_frame_name = 'base_link'

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.xline = tu.double_spin_box(pbox, -999., 999., .01) #QLineEdit(pbox)
        self.yline = tu.double_spin_box(pbox, -999., 999., .01) #QLineEdit(pbox)
        self.tline = tu.double_spin_box(pbox, -180., 180., .01) #QLineEdit(pbox)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.reset()

        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&theta", self.tline)
        formlayout.addRow(self.pose_button)
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        pbox.update()

    def get_current_pose(self):
        self.tf_listener.waitForTransform(self.default_frame, self.robot_frame_name, rospy.Time(), rospy.Duration(2.))
        p_base = tfu.transform(self.default_frame, self.robot_frame_name, self.tf_listener) \
                    * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
        t, r = tfu.matrix_as_tf(p_base)
        x = t[0]
        y = t[1]
        theta = tr.euler_from_quaternion(r)[2]
        
        self.xline.setValue(x)
        self.yline.setValue(y)
        self.tline.setValue(math.degrees(theta))

    def new_node(self, name=None):
        xy = [self.xline.value(), self.yline.value()]
        theta = math.radians(self.tline.value())
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        state = NavigateState(nname, xy, theta)
        return state

    def set_node_properties(self, node):
        xy = node.xy
        self.xline.setValue(xy[0])
        self.yline.setValue(xy[1])
        self.tline.setValue(math.degrees(node.theta))

    def reset(self):
        self.xline.setValue(0.)
        self.yline.setValue(0.)
        self.tline.setValue(0.)

#
# name maps to tool used to create it
# model
# is a state that can be stuffed into a state machine
class NavigateState(tu.SimpleStateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, xy, theta): #, frame):
        tu.SimpleStateBase.__init__(self, name, \
                'move_base', mm.MoveBaseAction, 
                goal_cb_str = 'ros_goal') 

        self.xy = xy
        self.theta = theta #stored as r internally
        #self.frame = frame

    def ros_goal(self, userdata, default_goal):
        g = mm.MoveBaseGoal()
        p = g.target_pose
        
        p.header.frame_id = 'map'
        p.header.stamp = rospy.get_rostime()
        p.pose.position.x = self.xy[0]
        p.pose.position.y = self.xy[1]
        p.pose.position.z = 0
        
        r = tr.quaternion_from_euler(0, 0, self.theta)
        p.pose.orientation.x = r[0]
        p.pose.orientation.y = r[1]
        p.pose.orientation.z = r[2]
        p.pose.orientation.w = r[3]
        return g




