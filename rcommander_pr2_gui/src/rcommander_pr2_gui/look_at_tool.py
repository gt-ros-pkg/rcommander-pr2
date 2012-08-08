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


#
# controller and view
# create and edits smach states
class LookAtTool(tu.ToolBase, p2u.SE3Tool):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'look_at', 'Look At', LookAtState)
        self.tf_listener = rcommander.tf_listener
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=False)


    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        group_boxes = self.make_se3_boxes(pbox)
        frame_box = self.make_task_frame_box(pbox)

        formlayout.addRow("&Frame", frame_box)
        formlayout.addRow(group_boxes[0])
        formlayout.addRow(group_boxes[1])
        pbox.update()
        self.reset()

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        pose_stamped = self.get_posestamped()
        state = LookAtState(nname, pose_stamped)
        return state

    def set_node_properties(self, node):
        self.set_posestamped(node.pose_stamped)

    def reset(self):
        for vr in [self.xline, self.yline, self.zline, self.phi_line, self.theta_line, self.psi_line]:
            vr.setValue(0.0)
        self.frame_box.setCurrentIndex(self.frame_box.findText('/task_frame'))


class LookAtState(tu.StateBase): 

    def __init__(self, name, pose_stamped):
        tu.StateBase.__init__(self, name)
        self.pose_stamped = pose_stamped

    def get_smach_state(self):
        return LookAtSmach(self.pose_stamped)


class LookAtSmach(smach.State): 

    def __init__(self, pose_stamped):
        smach.State.__init__(self, outcomes = ['succeeded', 'failed'], input_keys = [], output_keys = [])
        self.pose_stamped = pose_stamped
        #self.move_base_client = actionlib.SimpleActionClient('move_base', mm.MoveBaseAction)

    def set_robot(self, pr2):
        if pr2 != None:
            self.tf_listener = pr2.tf_listener
            self.head = pr2.head

    def execute(self, userdata):
        pt = np.matrix([self.pose_stamped.pose.position.x, self.pose_stamped.pose.position.y, self.pose_stamped.pose.position.z]).T
        if self.head.look_at(pt, frame=self.pose_stamped.header.frame_id):
            return 'succeeded'
        else:
            return 'failed'


    #def look_at(self, pt3d, frame='base_link', pointing_frame="wide_stereo_link",
    #            pointing_axis=np.matrix([1, 0, 0.]).T, wait=True):

        #print 'waiting for transform'
        #self.tf_listener.waitForTransform(self.pose_stamped.header.frame_id, '/map', rospy.Time(0), rospy.Duration(10.))
        #self.pose_stamped.header.stamp = rospy.Time()
        #ps = self.tf_listener.transformPose('/map', self.pose_stamped)
        #g = mm.MoveBaseGoal()
        #p = g.target_pose
        #
        #p.header.frame_id = ps.header.frame_id
        #p.header.stamp = rospy.get_rostime()

        #p.pose.position.x = ps.pose.position.x
        #p.pose.position.y = ps.pose.position.y
        #p.pose.position.z = 0 #ps.pose.position.z
       
        #raw_angles = tr.euler_from_quaternion([ps.pose.orientation.x, ps.pose.orientation.y,
        #                          ps.pose.orientation.z, ps.pose.orientation.w])
        #quat = tr.quaternion_from_euler(0., 0., raw_angles[2])
        #p.pose.orientation.x = quat[0]
        #p.pose.orientation.y = quat[1]
        #p.pose.orientation.z = quat[2]
        #p.pose.orientation.w = quat[3]

        #print g
        #self.move_base_client.send_goal(g)
        #result = tu.monitor_goals(self, [self.move_base_client], 'LookAtSmach', 60*60)
        #if result == 'failed':
        #    return 'aborted'
        #return result

