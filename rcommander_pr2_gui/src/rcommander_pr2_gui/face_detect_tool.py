import rcommander.tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import numpy as np
import smach
import rospy
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms, ClearTransforms
from face_detector.msg import *
import actionlib
import geometry_msgs.msg as geo
import rcommander.tf_utils as tfu
from object_manipulator.convert_functions import mat_to_pose, stamp_pose, change_pose_stamped_frame

## Tools that detects a face and creates a frame for it
class FaceDetectTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'face_detect', 'Detect Face', FaceDetectState)
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=True)
        self.clear_frames_service = rospy.ServiceProxy('clear_all_transforms', ClearTransforms, persistent=True)
        self.default_frame = '/base_link'
        self.tf_listener = rcommander.tf_listener

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        #action requires nothing..
        #time out? ok timeout
        self.time_out_box = QDoubleSpinBox(pbox)
        self.time_out_box.setMinimum(1.)
        self.time_out_box.setMaximum(1000.)
        self.time_out_box.setSingleStep(1.)

        self.frame_box = tu.FrameBox(self.frames_service)
        formlayout.addRow("&Orientation Frame", self.frame_box.create_box(pbox))
        formlayout.addRow('&Time Out', self.time_out_box)
        self.reset()

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        ori_frame = self.frame_box.text()
        time_out = self.time_out_box.value()
        return FaceDetectState(nname, ori_frame, time_out)

    ## Inherited
    def reset(self):
        self.frame_box.set_text(self.default_frame)
        self.time_out_box.setValue(120.)

    ## Inherited
    def set_node_properties(self, node):
        self.frame_box.set_text(node.orientation_frame)
        self.time_out_box.setValue(node.time_out)


class FaceDetectState(tu.StateBase):

    ## Constructor
    def __init__(self, name, orientation_frame, time_out):
        tu.StateBase.__init__(self, name)
        self.orientation_frame = orientation_frame
        self.time_out = time_out

    ## Inherited
    def get_smach_state(self):
        return FaceDetectFaceSmach(self.get_name(), self.orientation_frame, self.time_out)


class FaceDetectFaceSmach(tu.ActionWithTimeoutSmach):

    ## Constructor
    def __init__(self, frame_name, orientation_frame, time_out):
        tu.ActionWithTimeoutSmach.__init__(self, time_out, 'face_detector_action', FaceDetectorAction)
        self.broadcast_transform_srv = rospy.ServiceProxy('broadcast_transform', BroadcastTransform)
        self.orientation_frame = orientation_frame
        self.frame_name = frame_name

    ## Inherited
    def get_goal(self):
        return FaceDetectorGoal()

    ## Inherited
    def set_robot(self, robot):
        self.robot = robot

    ## Inherited
    def process_result(self, face_result):
        face = face_result.face_positions[0]
        point_stamped = geo.PointStamped()
        point_stamped.header.frame_id = face.header.frame_id
        point_stamped.point  = face.pos 
        frame_id_T_frame_name = tfu.origin_to_frame(point_stamped, self.orientation_frame, 
                                    self.robot.tf_listener, point_stamped.header.frame_id)
        ps = stamp_pose(mat_to_pose(frame_id_T_frame_name), point_stamped.header.frame_id)
        ps = change_pose_stamped_frame(self.robot.tf_listener, ps, '/base_link')
        self.broadcast_transform_srv(self.frame_name, ps)

