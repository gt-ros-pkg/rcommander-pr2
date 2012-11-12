import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import arm_navigation_msgs.msg as an
import actionlib
import actionlib_msgs.msg as am
import numpy as np
import smach
import functools as ft
import pypr2.pr2_utils as p2u

## Move the arm to a set of joint positions using arm navigation.
class SafeMoveArmTool(tu.ToolBase, p2u.JointTool):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'save_move', 'Safe Move', 
                SafeMoveArmState)
        p2u.JointTool.__init__(self, rcommander.robot, rcommander)
        self.shoulder_pan_joint = None 

    ## Callback for when a joint value changes
    def _value_changed_validate(self, value, joint):
        arm = self.get_arm_radio()
        self.check_joint_limits(arm, value, joint, (0,0,0,255))

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        fields, arm_radio_boxes, buttons = self.make_joint_boxes(pbox, 
                self.rcommander)
        formlayout.addRow('&Arm', arm_radio_boxes)
        for field in fields:
            formlayout.addRow(field['name'], field['item'])
            vchanged_func = ft.partial(self._value_changed_validate, 
                    joint=field['joint'])
            self.rcommander.connect(field['item'], 
                    SIGNAL('valueChanged(double)'), vchanged_func)

        for button in buttons:
            formlayout.addRow(button)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Update')
        formlayout.addRow(self.pose_button)

        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), 
                self.current_pose_cb)
        self.reset()

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        arm = tu.selected_radio_button(self.arm_radio_buttons).lower()
        joints = self.read_joints_from_fields(True)
        return SafeMoveArmState(nname, arm, joints)

    ## Inherited
    def set_node_properties(self, my_node):
        self.set_arm_radio(my_node.arm)
        self.set_joints_to_fields(my_node.joints)

    ## Inherited
    def reset(self):
        self.set_arm_radio('right')
        self.set_all_fields_to_zero()


class SafeMoveArmState(tu.StateBase): 

    ## Constructor
    # @param name Name of node.
    # @param arm 'left' or 'right'
    # @param joints A list of joint angles.
    def __init__(self, name, arm, joints):
        tu.StateBase.__init__(self, name)
        self.arm = arm
        self.joints = joints

    def get_smach_state(self):
        return SafeMoveArmStateSmach(self.arm, self.joints)

class SafeMoveArmStateSmach(smach.State): 

    ## Time out for execution.  Planner shouldn't take longer than this to
    #respond to us.
    TIME_OUT = 60

    ## Constructor
    def __init__(self, arm, joints):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 
            'failed', 'start_in_collision', 'goal_in_collision', 
            'joint_limit_violated'], input_keys = [], output_keys = [])
        self.joints = joints
        self.arm = arm

        if arm == 'left':
            self.move_arm_client = actionlib.SimpleActionClient('move_left_arm', 
                    an.MoveArmAction)
            self.joint_names = rospy.get_param('/l_arm_controller/joints')
            self.group_name = 'left_arm'

        if arm == 'right':
            self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', 
                    an.MoveArmAction)
            self.joint_names = rospy.get_param('/r_arm_controller/joints')
            self.group_name = 'right_arm'

    ## Inherited
    def set_robot(self, robot):
        if robot != None:
            self.controller_manager = robot.controller_manager

    ## Inherited
    def execute(self, userdata):        
        #Construct goal and send it
        status, started, stopped = self.controller_manager.joint_mode(self.arm)

        goal = an.MoveArmGoal()
        goal.motion_plan_request.group_name = self.group_name
        goal.motion_plan_request.num_planning_attempts = 2;
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        for (joint_name, joint_angle) in zip(self.joint_names, self.joints):
            joint_constraint = an.JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_angle
            joint_constraint.tolerance_below = .1
            joint_constraint.tolerance_above = .1
            goal.motion_plan_request.goal_constraints.joint_constraints.append(
                    joint_constraint) 
        self.move_arm_client.send_goal(goal)
    
        #Wait for action to finish
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        state = self.move_arm_client.get_state()
        preempted = False
        succeeded = False
        while not rospy.is_shutdown():
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('SafeMoveArmState: preempt requested')
                self.move_arm_client.cancel_goal()
                self.service_preempt()
                preempted = True
                break
            
            #we timed out
            if (rospy.get_time() - start_time) > SafeMoveArmStateSmach.TIME_OUT:
                self.move_arm_client.cancel_goal()
                rospy.loginfo('SafeMoveArmState: timed out!')
                break

            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                result = self.move_arm_client.get_result()
                if state == am.GoalStatus.SUCCEEDED:
                    if result.error_code.val == 1:
                        rospy.loginfo('SafeMoveArmState: Succeeded!')
                        succeeded = True
                elif result.error_code.val \
                        == an.ArmNavigationErrorCodes.START_STATE_IN_COLLISION:
                    return 'start_in_collision'
                elif result.error_code.val \
                        == an.ArmNavigationErrorCodes.GOAL_IN_COLLISION:
                    return 'goal_in_collision'
                elif result.error_code.val \
                        == an.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED:
                    return 'joint_limit_violated'
                rospy.loginfo('Got error code %d' % result.error_code.val)
                break

            state = self.move_arm_client.get_state()
            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'


        return 'failed'

