#import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import simple_move_base.msgs as smb
import math
import actionlib

def se2_from_se3(mat):
    t, r = tfu.matrix_as_tf(mat)
    x = t[0]
    y = t[1]
    theta = tr.euler_from_quaternion(r)[2]
    return x,y,t

#
# controller and view
# create and edits smach states
class PreciseNavigateTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate_refined', 'Navigate (precise)', PreciseNavigateTool)
        self.tf_listener = rcommander.tf_listener

        self.robot_frame_name = 'base_link'
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=True)
        gravity_aligned_frames = ['map', 'base_link']
        self.allowed_frames = []
        for f in self.frames_service().frames
            if f in gravity_aligned_frames:
                self.allowed_frames.append(f)


    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.xline = tu.double_spin_box(pbox, -999., 999., .01) #QLineEdit(pbox)
        self.yline = tu.double_spin_box(pbox, -999., 999., .01) #QLineEdit(pbox)
        self.tline = tu.double_spin_box(pbox, -180., 180., .01) #QLineEdit(pbox)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.reset()

        self.frame_box = QComboBox(pbox)
        for f in self.allowed_frames:
            frame_box.addItem(f)

        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&theta", self.tline)

        formlayout.addRow(self.pose_button)
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        pbox.update()

    def get_current_pose(self):
        frame = str(self.frame_box.currentText())
        self.tf_listener.waitForTransform(frame, self.robot_frame_name, rospy.Time(), rospy.Duration(2.))
        p_base = tfu.transform(frame, self.robot_frame_name, self.tf_listener) \
                    * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
        x, y, theta = se2_from_se3(p_base)
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
        state = PreciseNavigateTool(nname, xy, theta, str(self.frame_box.currentText()))
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


class PreciseNavigateState(tu.StateBase): 

    def __init__(self, name, x, y, theta, frame):
        tu.StateBase.__init__(self, name)
        self.x = x
        self.y = y
        self.t = t
        self.frame = frame

    def get_smach_state(self):
        return PreciseNavigateSmach(self.x, self.y, self.t, self.frame)


class PreciseNavigateSmach(smach.State): 

    def __init__(self, x, y, theta, frame):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = [], output_keys = [])
        self.go_angle_client = actionlib.SimpleActionClient('go_angle', smb.GoAngleAction)
        self.go_xy_client = actionlib.SimpleActionClient('go_xy', smb.GoXYAction)

        self.x = x
        self.y = y
        self.t = t
        self.frame = frame

    def set_robot(self, pr2):
        self.tf_listener = pr2.tf_listener

    def execute(self, userdata):
        #Create goal and send it up here
        bl_T_frame = tfu.tf_as_matrix(self.tf_listener.lookupTransform('base_link', self.frame, rospy.Time(0)))
        h_frame = tfu.tf_as_matrix(([self.x, self.y, 0], tr.quaternion_from_euler(0, 0, self.t)))
        x, y, t = se2_from_se3(tfu.matrix_as_tf(bl_T_frame * h_frame))

        xy_goal = smb.GoXYGoal(x,y)
        self.go_xy_client.send_goal(xy_goal)
        result_xy = tu.monitor_goals(self, [self.go_xy_client], 'PreciseNavigateSmach', self.timeout)
        if result_xy != 'succeeded':
            return result_xy

        ang_goal = smb.GoAngleGoal(ang_goal)
        self.go_angle_client.send_goal(ang_goal)
        result_ang = tu.monitor_goals(self, [self.go_angle_client], 'PreciseNavigateSmach', self.timeout)

        return result_ang
















    
    
    
    
    
    
    
    
    
    
    
    
    
    #def execute_goal(self, goal, action):
    #    action.send_goal(goal)

    #    succeeded = False
    #    preempted = False
    #    start_time = rospy.get_time()

    #    r = rospy.Rate(30)
    #    while not rospy.is_shutdown():
    #        #we have been preempted
    #        if self.preempt_requested():
    #            rospy.loginfo('PreciseNavigateSmach: preempt requested')
    #            self.service_preempt()
    #            action.cancel_goal()
    #            preempted = True
    #            break


    #        if (rospy.get_time() - start_time) > trajectory_time_out:
    #            action.cancel_goal() 
    #            rospy.loginfo('PreciseNavigateSmach: timed out!')
    #            succeeded = False
    #            break


##
## name maps to tool used to create it
## model
## is a state that can be stuffed into a state machine
#class PreciseNavigateTool(tu.SimpleStateBase): # smach_ros.SimpleActionState):
#
#    def __init__(self, name, xy, theta, frame): #, frame):
#        tu.SimpleStateBase.__init__(self, name, \
#                'go_xy', smb.GoXYAction, 
#                goal_cb_str = 'ros_goal') 
#        self.xy = xy
#        self.theta = theta 
#        self.frame = frame
#
#    def ros_goal(self, userdata, default_goal):
#        g = smb.GoXYGoal()
#        g.x
#        g.y
#
#        #p = g.target_pose
#        #
#        #p.header.frame_id = 'map'
#        #p.header.stamp = rospy.get_rostime()
#        #p.pose.position.x = self.xy[0]
#        #p.pose.position.y = self.xy[1]
#        #p.pose.position.z = 0
#        #
#        #r = tr.quaternion_from_euler(0, 0, self.theta)
#        #p.pose.orientation.x = r[0]
#        #p.pose.orientation.y = r[1]
#        #p.pose.orientation.z = r[2]
#        #p.pose.orientation.w = r[3]
#        return g




