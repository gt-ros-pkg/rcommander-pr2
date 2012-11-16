import roslib; roslib.load_manifest('pycontroller_manager')
import rospy
import pr2_mechanism_msgs.srv as pmm

LOADED_CTRLS_PARAMS = {
    'r' : "/controller_manager/loaded_ctrls/right_arm",
    'l' : "/controller_manager/loaded_ctrls/left_arm",
}

## Manages which controller should be running (joint or cartesian)
class ControllerManager:

    ## Constructor
    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy(\
                'pr2_controller_manager/load_controller', 
                pmm.LoadController)

        # UnloadController        
        self.unload = rospy.ServiceProxy(\
                'pr2_controller_manager/unload_controller', 
                pmm.UnloadController)

        # SwitchController
        self._switch_controller = rospy.ServiceProxy(\
                'pr2_controller_manager/switch_controller', 
                pmm.SwitchController)

        self.list_controllers = rospy.ServiceProxy(\
                'pr2_controller_manager/list_controllers', 
                pmm.ListControllers)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    ## Given the list of starting controllers, look for the possible
    # arm controllers which
    # conflict with those which are currently running.
    # @param start_con list of controllers that will be started
    def get_possible_running_ctrls(self, start_con):
        #rospy.loginfo("get_possible... in")
        # find the sides the start controllers are on
        arms_starting = []
        for con in start_con:
            for arm in ['l', 'r']:
                if con == self.joint_controllers[arm] \
                        or con == self.cart_controllers[arm]:
                    arms_starting.append(arm)

        # find the possible controllers running corresponding to those sides
        possible_ctrls = []
        for arm in ['l', 'r']:
            if arm in arms_starting:
                try:
                    possible_ctrls.extend(rospy.get_param(\
                            LOADED_CTRLS_PARAMS[arm]))
                except KeyError, e:
                    pass

        # check to see if the possible controllers are indeed running
        stop_con = []
        con_states = self.list_controllers()
        for i, controller in enumerate(con_states.controllers):
            if controller in possible_ctrls \
                    and con_states.state[i] == 'running':
                stop_con.append(controller)
        stop_con_no_start = []
        for con in stop_con:
            if not con in start_con:
                stop_con_no_start.append(con)
        return stop_con_no_start

    ## Switches between two sets of controllers
    # @param start_con list of arm controllers to start.
    # @param stop_con list of arm controllers to stop.
    # @return response from switch controller service, list of
    #       controllers started, list of controllers stopped
    def switch(self, start_con, stop_con):
        stop_con.extend(self.get_possible_running_ctrls(start_con)) # KelseyH

        con = self.list_controllers()
        valid_start = []
        valid_stop = []

        #Add the ones that are loaded but not running.
        for idx, controller in enumerate(con.controllers):
            if controller in start_con:
                if con.state[idx] != 'running':
                    valid_start.append(controller)

            if controller in stop_con:
                if con.state[idx] != 'stopped':
                    valid_stop.append(controller)

        #Add all controllers not loaded! But load them first
        for n in start_con:
            if not n in con.controllers:
                self.load(n)
                valid_start.append(n)

        if len(start_con) > 0:
            resp = self._switch_controller(valid_start, valid_stop, 
                    pmm.SwitchControllerRequest.STRICT)
            return resp.ok, valid_start, valid_stop
        else:
            return None, valid_start, valid_stop

    ## Puts the given arm in joint mode
    ## @param arm either 'left', 'right or 'both'
    def joint_mode(self, arm):
        #get current state
        statuses = True
        all_started = []
        all_stopped = []
        if arm == 'left' or arm == 'both':
            status, started, stopped = self.switch(
                    [self.joint_controllers['l']], 
                    [self.cart_controllers['l']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        if arm == 'right' or arm == 'both':
            status, started, stopped = self.switch(
                    [self.joint_controllers['r']], 
                    [self.cart_controllers['r']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        return statuses, all_started, all_stopped

    ## Puts the given arm in Cartesian mode
    ## @param arm either 'left', 'right or 'both'
    def cart_mode(self, arm):
        statuses = True
        all_started = []
        all_stopped = []
        if arm == 'left' or arm == 'both':
            #print 'switchleft'
            status, started, stopped = self.switch(
                    [self.cart_controllers['l']], 
                    [self.joint_controllers['l']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped
        if arm == 'right' or arm == 'both':
            #print 'switchright'
            status, started, stopped = self.switch(
                    [self.cart_controllers['r']], 
                    [self.joint_controllers['r']])
            statuses = status and statuses
            all_started += started
            all_stopped += stopped

        return statuses, all_started, all_stopped



