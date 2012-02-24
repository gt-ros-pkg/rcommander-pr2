import roslib; roslib.load_manifest('rcommander')
import rospy
import actionlib
import trajectory_msgs.msg as tm
import numpy as np
import functools as ft
import sensor_msgs.msg as sm
import std_msgs.msg as stdm
import pr2_controllers_msgs.msg as pm
import geometry_msgs.msg as gm
import geometry_msgs.msg as geo
import time
from kinematics_msgs.srv import GetKinematicSolverInfo
from pycontroller_manager.pycontroller_manager import ControllerManager
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tf_utils as tfu

def origin_to_frame(origin, supplement_frame, tf_listener, command_frame):
    m = origin
    if instanceof(m, geo.PointStamped):
        #point in some frame, needs orientation...
        CMD_T_pf = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
        p_CMD = CMD_T_pf * np.matrix([m.x, m.y, m.z, 1.]).T
        CMD_T_f = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, supplement_frame, rospy.Time(0)))
        CMD_T_frame = tll_T_f.copy()
        CMD_T_frame[0:3, 3] = p_CMD[0:3, 0]
    #If it's a pose stamped then we turn the pose stamped into a frame?
    elif instanceof(m, geo.PoseStamped):
        fid_T_p = pose_to_mat(m.pose)
        CMD_T_fid = tfu.tf_as_matrix(tf_listener.lookupTransform(COMMAND_FRAME, m.header.frame_id, rospy.Time(0)))
        CMD_T_frame = CMD_T_fid * fid_T_p
    else:
        raise RuntimeError('Got origin that is an instance of ' + str(m.__class__))

    return CMD_T_frame

class ListManager:

    def __init__(self, get_current_data_cb, set_current_data_cb, add_element_cb=None, name_preffix='point'):
        self.get_current_data_cb = get_current_data_cb
        self.set_current_data_cb = set_current_data_cb
        self.add_element_cb = add_element_cb
        self.name_preffix = name_preffix
        self.data_list = []

    def item_selection_changed_cb(self):
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return
        idx = self._find_index_of(str(selected[0].text()))
        self.curr_selected = idx
        self.set_current_data_cb(self.data_list[idx]['data'])

    def _find_index_of(self, name):
        for idx, tup in enumerate(self.data_list):
            if tup['name'] == name:
                return idx
        return None

    def make_widgets(self, parent, connector):
        self.list_box = QWidget(parent)
        self.list_box_layout = QHBoxLayout(self.list_box)
        self.list_box_layout.setMargin(0)

        self.list_widget = QListWidget(self.list_box)
        connector.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)
        self.list_box_layout.addWidget(self.list_widget)

        self.list_widget_buttons = QWidget(parent)
        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)

        self.move_up_button = QPushButton(self.list_widget_buttons)
        self.move_up_button.setText('Up')
        connector.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
        self.lbb_hlayout.addWidget(self.move_up_button)

        self.move_down_button = QPushButton(self.list_widget_buttons)
        self.move_down_button.setText('Down')
        connector.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
        self.lbb_hlayout.addWidget(self.move_down_button)

        spacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.lbb_hlayout.addItem(spacer)

        self.add_button = QPushButton(self.list_widget_buttons)
        self.add_button.setText('Add')
        connector.connect(self.add_button, SIGNAL('clicked()'), self.add_cb)

        self.remove_button = QPushButton(self.list_widget_buttons)
        self.remove_button.setText('Remove')
        connector.connect(self.remove_button, SIGNAL('clicked()'), self.remove_pose_cb)

        self.save_button = QPushButton(self.list_widget_buttons)
        self.save_button.setText('Save')
        connector.connect(self.save_button, SIGNAL('clicked()'), self.save_button_cb)

        self.lbb_hlayout.addWidget(self.add_button)
        self.lbb_hlayout.addWidget(self.remove_button)
        self.lbb_hlayout.addWidget(self.save_button)
        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)
        return [self.list_box, self.list_widget_buttons]


    def _refill_list_widget(self, data_list):
        self.list_widget.clear()
        for d in data_list:
            self.list_widget.addItem(d['name'])

    def _has_name(self, test_name):
        for rec in self.data_list:
            if rec['name'] == test_name:
                return True
            else:
                return False

    def _create_name(self):
        idx = len(self.data_list)
        tentative_name = self.name_preffix + ('%d' % idx)
        while self._has_name(tentative_name):
            idx = idx + 1
            tentative_name = self.name_preffix + ('%d' % idx)
        return tentative_name

    def add_cb(self):
        name = self._create_name()
        self.data_list.append({'name': name, 
                               'data': self.get_current_data_cb()})
        self._refill_list_widget(self.data_list)
        if self.add_element_cb != None:
            self.add_element_cb()

    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.data_list.pop(idx)
        self.data_list.insert(idx-1, item)

        #refresh
        self._refill_list_widget(self.data_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))

    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.data_list.pop(idx)
        self.data_list.insert(idx+1, item)

        #refresh
        self._refill_list_widget(self.data_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))

    def save_button_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        el = self.data_list[idx]
        self.data_list[idx] = {'name': el['name'],
                               'data': self.current_data_cb()}

    def remove_pose_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        if idx == None:
            raise RuntimeError('Inconsistency detected in list')
        else:
            self.data_list.pop(idx)
        self._refill_list_widget(self.data_list)

    def get_data(self):
        return self.data_list

    def set_data(self, data_list):
        self.data_list = data_list
        self._refill_list_widget(self.data_list)


def selected_radio_button(buttons_list):
    selected = None
    for r in buttons_list:
        if r.isChecked():
            selected = str(r.text())
    return selected

def position(point):
    return [point.x, point.y, point.z]

def quaternion(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

def combobox_idx(combobox, name):
    if name == None:
        name = ' '
    idx = combobox.findText(name)
    if idx == -1:
        combobox.addItem(name)
        idx = combobox.findText(name)
    return idx

#Test this
def unwrap2(cpos, npos):
    two_pi = 2*np.pi
    nin = npos % two_pi
    n_multiples_2pi = np.floor(cpos/two_pi)
    return nin + n_multiples_2pi*two_pi

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi

##
# Takes a normal ROS callback channel and gives it an on demand query style
# interface.
class GenericListener:
    ##
    # Message has to have a header
    # @param node_name name of node (if haven't been inited)
    # @param message_type type of message to listen for
    # @param listen_channel ROS channel to listen
    # @param frequency the frequency to expect messages (used to print warning statements to console)
    # @param message_extractor function to preprocess the message into a desired format
    # @param queue_size ROS subscriber queue (None = infinite)
    def __init__(self, node_name, message_type, listen_channel,
                 frequency, message_extractor=None, queue_size=None):
        #try:
            #print node_name, ': inited node.'
            #rospy.init_node(node_name, anonymous=True)
        #except rospy.ROSException, e:
        #    pass
        self.last_msg_returned   = None   #Last message returned to callers from this class
        self.last_call_back      = None   #Local time of last received message
        self.delay_tolerance     = 1/frequency #in seconds
        self.reading             = {'message':None, 'msg_id':-1}
        self.curid               = 0
        self.message_extractor = message_extractor

        def callback(*msg):
            #If this is a tuple (using message filter)
            if 'header' in dir(msg):
                if msg.__class__ == ().__class__:
                    msg_number = msg[0].header.seq
                else:
                    msg_number = msg.header.seq
            else:
                msg_number = self.curid
                self.curid += 1

            #*msg makes everything a tuple.  If length is one, msg = (msg, )
            if len(msg) == 1:
                msg = msg[0]
            
            self.reading  = {'message':msg, 'msg_id':msg_number}

            #Check for delayed messages
            self.last_call_back = time.time() #record when we have been called back last

        if message_type.__class__ == [].__class__:
            import message_filters
            subscribers = [message_filters.Subscriber(channel, mtype) for channel, mtype in zip(listen_channel, message_type)]
            queue_size = 10
            ts = message_filters.TimeSynchronizer(subscribers, queue_size)
            ts.registerCallback(callback)
        else:
            rospy.Subscriber(listen_channel, message_type, callback,
                             queue_size = queue_size)

        self.node_name = node_name
        #print node_name,': subscribed to', listen_channel
        rospy.loginfo('%s: subscribed to %s' % (node_name, listen_channel))

    def _check_for_delivery_hiccups(self):
        #If have received a message in the past
        if self.last_call_back != None:
            #Calculate how it has been
            time_diff = time.time() - self.last_call_back
            #If it has been longer than expected hz, complain
            if time_diff > self.delay_tolerance:
                print self.node_name, ': have not heard back from publisher in', time_diff, 's'

    def _wait_for_first_read(self, quiet=False):
        if not quiet:
            rospy.loginfo('%s: waiting for reading ...' % self.node_name)
        while self.reading['message'] == None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            #if not quiet:
            #    print self.node_name, ': waiting for reading ...'

    ## 
    # Supported use cases
    # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
    # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
    # ft     - want to get a reading, can be stale, duplication allowed    (don't want a None), query speed important
    # NOT ALLOWED                                   duplication allowed,                        willing to wait for new data
    def read(self, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True):
        if allow_duplication:
            if willing_to_wait:
                raise RuntimeError('Invalid settings for read.')
            else: 
                # ft - want to get a reading, can be stale, duplication allowed (but don't want a None), query speed important
                #self._wait_for_first_read(quiet)
                reading                = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
        else:
            if willing_to_wait:
                # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
                self._wait_for_first_read(quiet)
                while self.reading['msg_id'] == self.last_msg_returned and not rospy.is_shutdown():
                    if warn:
                        self._check_for_delivery_hiccups()
                    rospy.sleep(1/1000.0)
                reading = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
            else:
                # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
                if self.last_msg_returned == self.reading['msg_id']:
                    return None
                else:
                    reading = self.reading
                    self.last_msg_returned = reading['msg_id']
                    if self.message_extractor is not None:
                        return self.message_extractor(reading['message'])
                    else:
                        return reading['message']


class Joint:

    def __init__(self, name, joint_provider):
        self.joint_provider = joint_provider
        self.joint_names = rospy.get_param('/%s/joints' % name)
        self.pub = rospy.Publisher('%s/command' % name, tm.JointTrajectory)
        self.names_index = None
        self.zeros = [0 for j in range(len(self.joint_names))]

    def pose(self, joint_states=None):
        #print 'POSE!!!'
        if joint_states == None:
            joint_states = self.joint_provider()
        #print 'JOINT STATE'

        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(joint_states.name):
                self.names_index[n] = i
            self.joint_idx = [self.names_index[n] for n in self.joint_names]
        #print 'GOT POSE. RETURNING'

        return (np.matrix(joint_states.position).T)[self.joint_idx, 0]

    def _create_trajectory(self, pos_mat, times, vel_mat=None):
        #Make JointTrajectoryPoints
        points = [tm.JointTrajectoryPoint() for i in range(pos_mat.shape[1])]
        for i in range(pos_mat.shape[1]):
            points[i].positions = pos_mat[:,i].A1.tolist()
            points[i].accelerations = self.zeros
            if vel_mat == None:
                points[i].velocities = self.zeros
            else:
                points[i].velocities = vel_mat[:,i].A1.tolist()

        for i in range(pos_mat.shape[1]):
            points[i].time_from_start = rospy.Duration(times[i])

        #Create JointTrajectory
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points = points
        jt.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        return jt

    def set_poses(self, pos_mat, times):
        #pos_mat = np.column_stack([self.pose(), pos_mat])
        #times = [0] + times
        #times = times + .1
        joint_trajectory = self._create_trajectory(pos_mat, times)
        print 'Sending message', joint_trajectory
        self.pub.publish(joint_trajectory)

    def get_joint_names(self):
        return self.joint_names


class PR2Arm(Joint):

    def __init__(self, joint_provider, tf_listener, arm):
        joint_controller_name = arm + '_arm_controller'
        cart_controller_name = arm + '_cart'
        Joint.__init__(self, joint_controller_name, joint_provider)
        self.arm = arm
        self.tf_listener = tf_listener
        #print 'PR2ARM; called CONTROLLERS'
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % joint_controller_name, pm.JointTrajectoryAction)
        self.joint_controller_name = joint_controller_name

        self.cart_posure_pub = rospy.Publisher("/%s/command_posture" % cart_controller_name, stdm.Float64MultiArray).publish
        self.cart_pose_pub = rospy.Publisher("/%s/command_pose" % cart_controller_name, gm.PoseStamped).publish
        if arm == 'l':
            self.full_arm_name = 'left'
        else:
            self.full_arm_name = 'right'


        #if use_kinematics:
            #self.kinematics = pr2k.PR2ArmKinematics(self.full_arm_name, self.tf_listener)
        #self.ik_utilities = iku.IKUtilities(self.full_arm_name, self.tf_listener) 
        self.limits_dict, self.vel_limit_dict = self._limits()

        self.POSTURES = {
            'off':          np.matrix([]),
            'mantis':       np.matrix([0, 1, 0,  -1, 3.14, -1, 3.14]).T,
            'elbowupr':     np.matrix([-0.79,0,-1.6,  9999, 9999, 9999, 9999]).T,
            'elbowupl':     np.matrix([0.79,0,1.6 , 9999, 9999, 9999, 9999]).T,
            'old_elbowupr': np.matrix([-0.79,0,-1.6, -0.79,3.14, -0.79,5.49]).T,
            'old_elbowupl': np.matrix([0.79,0,1.6, -0.79,3.14, -0.79,5.49]).T,
            'elbowdownr':   np.matrix([-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, -1.5498884526859626]).T, 
            'elbowdownl':   np.matrix([-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, -1.5565279256852611]).T
            }

    def get_limits(self):
        return self.limits_dict

    def get_vel_limits(self):
        return self.vel_limit_dict

    def _limits(self):
        service_name = '/pr2_%s_arm_kinematics/get_ik_solver_info' % self.full_arm_name
        print service_name
        proxy = rospy.ServiceProxy(service_name, GetKinematicSolverInfo)
        #import pdb
        #pdb.set_trace()
        info = proxy().kinematic_solver_info
        limit_dict = {}
        vel_limit_dict = {}
        for idx, name in enumerate(info.joint_names):
            limit = info.limits[idx]
            if limit.has_position_limits:
                limit_dict[name] = [limit.min_position, limit.max_position]
            if limit.has_velocity_limits:
                vel_limit_dict[name] = limit.max_velocity
        return limit_dict, vel_limit_dict

    def set_posture(self, posture_mat):
        self.cart_posure_pub(stdm.Float64MultiArray(data=posture_mat.A1.tolist()))

    ##
    # Send a cartesian pose to *_cart controllers
    # @param trans len 3 list
    # @param rot len 3 list
    # @param frame string
    # @param msg_time float
    def set_cart_pose(self, trans, rot, frame, msg_time):
        ps = gm.PoseStamped()
        for i, field in enumerate(['x', 'y', 'z']):
            exec('ps.pose.position.%s = trans[%d]' % (field, i))
        for i, field in enumerate(['x', 'y', 'z', 'w']):
            exec('ps.pose.orientation.%s = rot[%d]' % (field, i))
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time(msg_time)
        self.cart_pose_pub(ps)

    ##
    # @param pos_mat column matrix of poses
    # @param times array of times
    def set_poses(self, pos_mat, times, vel_mat=None, block=True):
        #p = self.pose()
        #for i in range(pos_mat.shape[1]):
        #    pos_mat[4,i] = unwrap2(p[4,0], pos_mat[4,i])
        #    pos_mat[6,i] = unwrap2(p[6,0], pos_mat[6,i])
        #    p = pos_mat[:,i]

        pos_mat = np.column_stack([self.pose(), pos_mat])
        #print 'SETPOSES', times, times.__class__
        times   = np.concatenate(([0], times))
        times = times + .1
        #print "SET POSES", pos_mat.shape, len(times)
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        #joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(5.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        #g.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def stop_trajectory_execution(self):
        self.client.cancel_all_goals()

    def has_active_goal(self):
        s = self.client.get_state()
        if s == amsg.GoalStatus.ACTIVE or s == amsg.GoalStatus.PENDING:
            return True
        else:
            return False

    def set_poses_monitored(self, pos_mat, times, vel_mat=None, block=True, time_look_ahead=.050):
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        #joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def set_pose(self, pos, nsecs=5., block=True):
        for i in range(2):
            cpos = self.pose()
        pos[4,0] = unwrap(cpos[4,0], pos[4,0])
        pos[6,0] = unwrap(cpos[6,0], pos[6,0])
        self.set_poses(np.column_stack([pos]), np.array([nsecs]), block=block)
        #self.set_poses(np.column_stack([cpos, pos]), np.array([min_time, min_time+nsecs]), block=block)

    def pose_cartesian(self, frame='base_link'):
        gripper_tool_frame = self.arm + '_gripper_tool_frame'
        return tfu.transform(frame, gripper_tool_frame, self.tf_listener)

    def pose_cartesian_tf(self, frame='base_link'):
        p, r = tfu.matrix_as_tf(self.pose_cartesian(frame))
        return np.matrix(p).T, np.matrix(r).T


class PR2Torso(Joint):

    def __init__(self, joint_provider):
        Joint.__init__(self, 'torso_controller', joint_provider)
        self.torso = actionlib.SimpleActionClient('torso_controller/position_joint_action', pm.SingleJointPositionAction)
        rospy.loginfo('waiting for torso_controller')
        self.torso.wait_for_server()

    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()


class PR2Head(Joint):

    def __init__(self, joint_provider):
        Joint.__init__(self, 'head_traj_controller', joint_provider)

    def set_pose(self, pos, length=5., block=False):
        for i in range(2):
            cpos = self.pose()
        MIN_TIME = .1
        self.set_poses(np.column_stack([cpos, pos]), np.array([MIN_TIME, MIN_TIME+length]))

class PR2:

    def __init__(self, tf_listener):
        #print '===================================='
        #print 'PR2 OBJECT CREATED'
        #print '===================================='
        self.tf_listener = tf_listener
        jl = GenericListener('joint_state_listener', sm.JointState, 'joint_states', 100)
        joint_provider = ft.partial(jl.read, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
        self.left = PR2Arm(joint_provider, tf_listener, 'l')
        self.right = PR2Arm(joint_provider, tf_listener, 'r')
        self.torso = PR2Torso(joint_provider)
        self.head = PR2Head(joint_provider)
        self.controller_manager = ControllerManager()

