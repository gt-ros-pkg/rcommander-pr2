import rcommander.tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import pypr2.tf_utils as tfu
import tf.transformations as tr
import move_base_msgs.msg as mm
import math
import pypr2.pr2_utils as p2u
import numpy as np
import smach
import actionlib
from tf_broadcast_server.srv import GetTransforms

## Points the robot's head at a 3D point defined in some frame.  Used for
# keeping some object of interest in the field-of-view of the robot's sensors.
# TODO: LookAt shouldn't use full 6D poses, only 3D will suffice.
class LookAtTool(tu.ToolBase, p2u.SE3Tool):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'look_at', 'Look At', LookAtState)
        self.tf_listener = rcommander.tf_listener
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=False)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        group_boxes = self.make_se3_boxes(pbox)
        frame_box = self.make_task_frame_box(pbox)

        formlayout.addRow("&Frame", frame_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])
        pbox.update()
        self.reset()

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        pose_stamped = self.get_posestamped()
        state = LookAtState(nname, pose_stamped)
        return state

    ## Inherited
    def set_node_properties(self, node):
        self.set_posestamped(node.pose_stamped)

    ## Inherited
    def reset(self):
        for vr in [self.xline, self.yline, self.zline, self.phi_line, \
                self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.frame_box.setCurrentIndex(self.frame_box.findText('/task_frame'))

class LookAtState(tu.StateBase): 

    ## Constructor
    # @param name Name of node
    # @param pose_stamped PoseStamped object representing the 3D point
    # to look at.
    def __init__(self, name, pose_stamped):
        tu.StateBase.__init__(self, name)
        self.pose_stamped = pose_stamped

    ## Inherited
    def get_smach_state(self):
        return LookAtSmach(self.pose_stamped)


class LookAtSmach(smach.State): 

    ## Constructor
    def __init__(self, pose_stamped):
        smach.State.__init__(self, outcomes = ['succeeded', 'failed'], 
                input_keys = [], output_keys = [])
        self.pose_stamped = pose_stamped

    ## Inherited
    def set_robot(self, pr2):
        if pr2 != None:
            self.tf_listener = pr2.tf_listener
            self.head = pr2.head

    ## Inherited
    def execute(self, userdata):
        pt = np.matrix([self.pose_stamped.pose.position.x, self.pose_stamped.pose.position.y, 
            self.pose_stamped.pose.position.z]).T
        if self.head.look_at(pt, frame=self.pose_stamped.header.frame_id):
            return 'succeeded'
        else:
            return 'failed'
