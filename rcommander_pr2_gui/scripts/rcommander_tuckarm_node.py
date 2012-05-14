#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rospy
from pycontroller_manager.pycontroller_manager import ControllerManager
import pr2_common_action_msgs.msg as ca 
import actionlib
import rcommander_pr2_gui.msg as rm
import pr2_common_action_msgs.msg as ca 
import actionlib_msgs.msg as am

class RCommanderTuckArmNode:
    def __init__(self):
        #Setup an action server where we call the real tuck arm
        self.tuckarm_as = actionlib.SimpleActionServer('rcommander_tuckarms', rm.RCTuckArmsAction, 
					                    execute_cb=self.action_cb, auto_start=False)
        self.tuckarm_as.start()
        self.tuckarm_client = actionlib.SimpleActionClient('tuck_arms', ca.TuckArmsAction)
        self.controller_manager = ControllerManager()


    def action_cb(self, request):
        #Make sure we're using the joint controllers
        rospy.loginfo('starting joint mode')
        arm = 'both'
        #self.controller_manager.joint_mode(arm)
        
        rospy.loginfo('sent goal')
        goal = ca.TuckArmsGoal()
        goal.tuck_left = request.tuck_left
        goal.tuck_right = request.tuck_right
        self.tuckarm_client.send_goal(goal)

        r = rospy.Rate(15)
        state = None
        while not rospy.is_shutdown():
            if self.tuckarm_as.is_preempt_requested():
                rospy.loginfo('goal canceled')
                self.tuckarm_client.cancel_goal()
                self.tuckarm_as.set_preempted()
                return

            state = self.tuckarm_client.get_state()
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                rospy.loginfo('finished goal')
                break 
            r.sleep()

        rospy.loginfo('Waiting for result from tuck arm...')
        self.tuckarm_client.wait_for_result()

        rospy.loginfo('Got result!')
        tuck_result = self.tuckarm_client.get_result()
        rcresult = rm.RCTuckArmsResult()
        rcresult.tuck_left = tuck_result.tuck_left
        rcresult.tuck_right = tuck_result.tuck_right

        if state == am.GoalStatus.SUCCEEDED:
            self.tuckarm_as.set_succeeded(rcresult)
        elif state == am.GoalStatus.PREEMPTED:
            self.tuckarm_as.set_preempted()
        else:
            self.tuckarm_as.set_aborted(rcresult)

if __name__ == '__main__':
    rospy.init_node('RCTuckArms')
    tuck_node = RCommanderTuckArmNode()
    rospy.loginfo('RC TuckArms action server started.')
    rospy.spin()





