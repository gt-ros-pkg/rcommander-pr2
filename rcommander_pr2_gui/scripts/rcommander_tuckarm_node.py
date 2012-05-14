#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rospy
from pycontroller_manager.pycontroller_manager import ControllerManager
import pr2_common_action_msgs.msg as ca 
import actionlib
import rcommander_pr2_gui.msg as rm
import pr2_common_action_msgs.msg as ca 
import actionlib_msgs.msg as am

l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
l_arm_untucked = [ 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_untucked = [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_approach = [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369]
r_arm_up_traj = [[-0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]


class RCommanderTuckArmNode:
    def __init__(self):
        #Setup an action server where we call the real tuck arm
        self.tuckarm_as = actionlib.SimpleActionServer('rcommander_tuckarms', rm.RCTuckArmsAction, 
					                    execute_cb=self.action_cb, auto_start=False)
        self.controller_manager = ControllerManager()

        #self.tuckarm_as.start()
        #self.tuckarm_client = actionlib.SimpleActionClient('tuck_arms', ca.TuckArmsAction)


    def action_cb(self, request):
        request.tuck_left


        status, started, stopped = self.controller_manager.joint_mode(self.arm)
        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
        client = self.arm_obj.client
        state = client.get_state()



if __name__ == '__main__':
    rospy.init_node('RCTuckArms')
    tuck_node = RCommanderTuckArmNode()
    rospy.loginfo('RC TuckArms action server started.')
    rospy.spin()





        ##Make sure we're using the joint controllers
        #rospy.loginfo('starting joint mode')
        #arm = 'both'
        ##self.controller_manager.joint_mode(arm)
        #
        #rospy.loginfo('sent goal')
        #goal = ca.TuckArmsGoal()
        #goal.tuck_left = request.tuck_left
        #goal.tuck_right = request.tuck_right
        #self.tuckarm_client.send_goal(goal)

        #r = rospy.Rate(15)
        #state = None
        #while not rospy.is_shutdown():
        #    if self.tuckarm_as.is_preempt_requested():
        #        rospy.loginfo('goal canceled')
        #        self.tuckarm_client.cancel_goal()
        #        self.tuckarm_as.set_preempted()
        #        return

        #    state = self.tuckarm_client.get_state()
        #    if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
        #        rospy.loginfo('finished goal')
        #        break 
        #    r.sleep()

        #rospy.loginfo('Waiting for result from tuck arm...')
        #self.tuckarm_client.wait_for_result()

        #rospy.loginfo('Got result!')
        #tuck_result = self.tuckarm_client.get_result()
        #rcresult = rm.RCTuckArmsResult()
        #rcresult.tuck_left = tuck_result.tuck_left
        #rcresult.tuck_right = tuck_result.tuck_right

        #if state == am.GoalStatus.SUCCEEDED:
        #    self.tuckarm_as.set_succeeded(rcresult)
        #elif state == am.GoalStatus.PREEMPTED:
        #    self.tuckarm_as.set_preempted()
        #else:
        #    self.tuckarm_as.set_aborted(rcresult)
