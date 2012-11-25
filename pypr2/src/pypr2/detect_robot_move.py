import sensor_msgs.msg as sm
import numpy as np
import copy
import rospy
import pr2_utils as pru


def get_joint_group(group_name):
    all_except_arms =  ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
                        'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint',
                        'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
                        'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint',
                        'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
                        'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
                        'torso_lift_joint', 'torso_lift_motor_screw_joint', 
                        'head_pan_joint', 'head_tilt_joint']
    
    joints_on_left_arm = ['l_' + n for n in pru.JOINT_NAME_FIELDS]
    
    joints_on_right_arm = ['r_' + n for n in pru.JOINT_NAME_FIELDS]

    if group_name == 'left_arm':
        return joints_on_left_arm
    elif group_name == 'right_arm':
        return joints_on_right_arm
    elif group_name == 'all_except_arms':
        return all_except_arms
    else:
        return None
        


## Detects whether any joint on the robot is moving.  Used for
# detecting when to stop updating AR ToolKit tag position.
class DetectRobotMove:

    ## Constructor
    # @param list_of_joints_to_watch List of lists of strings with 
    #       joint names (as declared by /joint_states).
    # @apram time_to_wait_after_moving Time to wait after moving before 
    #       declaring that robot is no longer moving
    def __init__(self, list_of_joints_to_watch, time_to_wait_after_moving=3.):
        self.last_msg = None
        self.joint_dict = {}
        self.is_moving_time = rospy.get_rostime().to_time()
        self.watch_idx = []
        self.list_of_joints_to_watch = list_of_joints_to_watch
        self.time_to_wait_after_moving = time_to_wait_after_moving

        ## Joints to watch for movements
        sub = rospy.Subscriber("joint_states", sm.JointState, 
                self.joint_state_cb)

    ## Callback whenever we receive a joint state message
    # @param msg a JointState message.
    def joint_state_cb(self, msg):
        if self.last_msg == None:
            self.last_msg = msg
            for idx, n in enumerate(msg.name):
                self.joint_dict[n] = idx
            for n in self.list_of_joints_to_watch:
                self.watch_idx.append(self.joint_dict[n])

        vels = np.array(copy.deepcopy(msg.velocity))
        nlist = np.array(msg.name)
        moving = np.abs(vels) > 0.01
        watch_joints = moving[self.watch_idx]
        watch_joints_n = nlist[self.watch_idx] 

        if watch_joints.any():
            self.is_moving_time = msg.header.stamp.to_time()
    
    ## Checks to see if the robot is moving
    # @return boolean
    def is_moving(self):
        return (rospy.get_rostime().to_time() - self.is_moving_time) \
                < self.time_to_wait_after_moving

    ## Gets the time that movement was last observed
    # @return time as float in seconds
    def last_moved_at_time(self):
        return self.is_moving_time

