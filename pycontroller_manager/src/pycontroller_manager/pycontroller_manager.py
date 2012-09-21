import roslib; roslib.load_manifest('pycontroller_manager')
import rospy
import pr2_mechanism_msgs.srv as pmm

LOADED_CTRLS_PARAMS = {
    'r' : "/controller_manager/loaded_ctrls/right_arm",
    'l' : "/controller_manager/loaded_ctrls/left_arm",
}

class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)

        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)

        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

        self.list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', pmm.ListControllers)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    ##
    # Given the list of starting controllers, look for the possible arm controllers which
    # conflict with those which are currently running.
    # KelseyH
    def get_possible_running_ctrls(self, start_con):
        #rospy.loginfo("get_possible... in")
        # find the sides the start controllers are on
        arms_starting = []
        for con in start_con:
            for arm in ['l', 'r']:
                if con == self.joint_controllers[arm] or con == self.cart_controllers[arm]:
                    arms_starting.append(arm)

        # find the possible controllers running corresponding to those sides
        possible_ctrls = []
        for arm in ['l', 'r']:
            if arm in arms_starting:
                try:
                    possible_ctrls.extend(rospy.get_param(LOADED_CTRLS_PARAMS[arm]))
                except KeyError, e:
                    pass

        # check to see if the possible controllers are indeed running
        stop_con = []
        con_states = self.list_controllers()
        for i, controller in enumerate(con_states.controllers):
            if controller in possible_ctrls and con_states.state[i] == 'running':
                stop_con.append(controller)
        stop_con_no_start = []
        for con in stop_con:
            if not con in start_con:
                stop_con_no_start.append(con)
        #rospy.loginfo("get_possible... out")
        #rospy.loginfo(str(arms_starting) + str(start_con) + str(possible_ctrls) + str(stop_con_no_start))
        return stop_con_no_start

    def switch(self, start_con, stop_con):
        stop_con.extend(self.get_possible_running_ctrls(start_con)) # KelseyH

        con = self.list_controllers()
        valid_start = []
        valid_stop = []

        #Add the ones that are loaded but not running.
        for idx, controller in enumerate(con.controllers):
            if controller in start_con:
                #print 'found', controller, 'it\'s state is', con.state[idx], len(con.state[idx]), con.state[idx].__class__
                if con.state[idx] != 'running':
                    #print 'ControllerManager: adding controller', controller
                    valid_start.append(controller)
                #else:
                #    print 'ControllerManager: not adding', controller

            if controller in stop_con:
                if con.state[idx] != 'stopped':
                    valid_stop.append(controller)

            #print controller, con.state[idx]

        #Add all controllers not loaded! But load them first
        for n in start_con:
            if not n in con.controllers:
                #print 'loading controller', n
                self.load(n)
                valid_start.append(n)

        if len(start_con) > 0:
            #print 'ControllerManager: starting', valid_start, 'stopping', valid_stop
            resp = self._switch_controller(valid_start, valid_stop, pmm.SwitchControllerRequest.STRICT)
            return resp.ok, valid_start, valid_stop
        else:
            #print 'ControllerManager: not starting or stopping any controllsers'
            return None, valid_start, valid_stop

    def joint_mode(self, arm):
        #get current state
        statuses = True
        all_started = []
        all_stopped = []
        if arm == 'left' or arm == 'both':
            status, started, stopped = self.switch([self.joint_controllers['l']], [self.cart_controllers['l']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        if arm == 'right' or arm == 'both':
            status, started, stopped = self.switch([self.joint_controllers['r']], [self.cart_controllers['r']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        return statuses, all_started, all_stopped

    def cart_mode(self, arm):
        statuses = True
        all_started = []
        all_stopped = []
        if arm == 'left' or arm == 'both':
            #print 'switchleft'
            status, started, stopped = self.switch([self.cart_controllers['l']], [self.joint_controllers['l']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped
        if arm == 'right' or arm == 'both':
            #print 'switchright'
            status, started, stopped = self.switch([self.cart_controllers['r']], [self.joint_controllers['r']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        return statuses, all_started, all_stopped



