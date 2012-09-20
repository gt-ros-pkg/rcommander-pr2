import sensor_msgs.msg as sm
import numpy as np
import copy
import rospy


class DetectRobotMove:

    JOINT_TO_WATCH = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
                        'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint',
                        'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
                        'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint',
                        'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
                        'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
                        'torso_lift_joint', 'torso_lift_motor_screw_joint', 
                        'head_pan_joint', 'head_tilt_joint']
    #CONSTANTLY_MOVING_JOINTS = ['laser_tilt_mount_joint', 'l_upper_arm_roll_joint', 'r_upper_arm_roll_joint', 'r_gripper_motor_slider_joint', 'l_gripper_motor_slider_joint']
    TIME_TO_WAIT_AFTER_MOVING = 3.

    def __init__(self):
        self.last_msg = None
        self.joint_dict = {}
        self.is_moving_time = rospy.get_rostime().to_time()
        self.watch_idx = []
        sub = rospy.Subscriber("joint_states", sm.JointState, self.joint_state_cb)

    def joint_state_cb(self, msg):
        if self.last_msg == None:
            self.last_msg = msg
            for idx, n in enumerate(msg.name):
                self.joint_dict[n] = idx
            for n in DetectRobotMove.JOINT_TO_WATCH:
                self.watch_idx.append(self.joint_dict[n])

        vels = np.array(copy.deepcopy(msg.velocity))
        nlist = np.array(msg.name)
        moving = np.abs(vels) > 0.01
        watch_joints = moving[self.watch_idx]
        watch_joints_n = nlist[self.watch_idx] 

        if watch_joints.any():
            self.is_moving_time = msg.header.stamp.to_time()

    def is_moving(self):
        return (rospy.get_rostime().to_time() - self.is_moving_time) < DetectRobotMove.TIME_TO_WAIT_AFTER_MOVING

