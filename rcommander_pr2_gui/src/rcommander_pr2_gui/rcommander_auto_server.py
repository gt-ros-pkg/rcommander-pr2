import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rospy
import actionlib

from pr2_object_manipulation_msgs.srv import ActionInfo, ActionInfoResponse
from pr2_object_manipulation_msgs.msg import *
import rcommander.graph_model as gm

from PyQt4 import QtCore, QtGui
import sys
import signal
import os
import os.path as pt
import pdb

class WatchDirectory(QtCore.QObject):
    def __init__(self, path, dir_changed_func=None, file_changed_func=None):
        self.path = path
        self.fs_watcher = QtCore.QFileSystemWatcher([path])
        self.fs_watcher.connect(self.fs_watcher, QtCore.SIGNAL('directoryChanged(QString)'), self.directory_changed)
        self.fs_watcher.connect(self.fs_watcher, QtCore.SIGNAL('fileChanged(QString)'), self.file_changed)
        self.file_changed_func = file_changed_func
        self.dir_changed_func = dir_changed_func

    def __del__(self):
        rospy.loginfo('Deleting watcher for ' + self.path)
        self.fs_watcher.disconnect(self.fs_watcher, 
                QtCore.SIGNAL('directoryChanged(QString)'), self.directory_changed)
        self.fs_watcher.disconnect(self.fs_watcher, 
                QtCore.SIGNAL('fileChanged(QString)'), self.file_changed)

    @QtCore.pyqtSlot(str)
    def directory_changed(self, path):
        if self.dir_changed_func != None:
            self.dir_changed_func(path)
    
    @QtCore.pyqtSlot(str)
    def file_changed(self, path):
        if self.file_changed_func != None:
            self.file_changed_func(path)

class RCommanderAutoServer:

    def __init__(self, robot, path_to_rcommander_files):
        self.path_to_rcommander_files = path_to_rcommander_files
        self.robot = robot
        self.action_dict = {}
        self.main_dir_watcher = WatchDirectory(self.path_to_rcommander_files, self.main_directory_changed)
        self.main_directory_changed(self.path_to_rcommander_files)

        rospy.Service('list_rcommander_actions', ActionInfo, self.list_action_cb)
        self.actserv = actionlib.SimpleActionServer('run_rcommander_action', RunScriptAction, execute_cb=self.execute_cb, auto_start=False)
        self.actserv.start()

    def execute_cb(self, goal):
        rospy.loginfo('Requested: group ' + goal.group_name + ' action: ' + goal.action_name)
        self.action_dict[goal.action_name]['server'].execute(self.actserv)

    def _find_all_actions(self):
        dirs = [] 
        for d in os.listdir(self.path_to_rcommander_files):
            if os.path.isdir(os.path.join(self.path_to_rcommander_files, d)):
                dirs.append(d)
        return dirs

    def _load(self, action):
        rospy.loginfo('Loading ' + action)
        action_path = os.path.join(self.path_to_rcommander_files, action)
        for i in range(4):
            try:
                load_dict = {'server':  ScriptedActionServer(action, action_path, self.robot),
                             'watcher': WatchDirectory(action_path, self.action_directory_changed)}
                rospy.loginfo('Successfully loaded ' + action)
                return load_dict
            except IOError, e:
                rospy.loginfo(str(e))
                rospy.loginfo('IOError encountered, retrying.')
                rospy.sleep(3)

    def action_directory_changed(self, action_path_name):
        action_path_name = str(action_path_name)
        action_name = pt.split(action_path_name)[1]
        rospy.loginfo('action_name ' + action_name)
        rospy.loginfo('action_directory_changed: ' + action_name)
        self.action_dict[action_name] = self._load(action_name)

    def main_directory_changed(self, main_path_name):
        rospy.loginfo('main_directory_changed: rescanning ' + main_path_name)
        ndict = {}
        actions = self._find_all_actions()
        for action in actions:
            if self.action_dict.has_key(action):
                ndict[action] = self.action_dict[action]
            else:
                loaded_action = self._load(action)
                if loaded_action != None:
                    ndict[action] = loaded_action
                else:
                    rospy.loginfo('Failed to load ' + action)
        self.action_dict = ndict

    def list_action_cb(self, req):
        actions = self.action_dict.keys()
        actions.sort()
        return ActionInfoResponse(actions)

class ScriptedActionServer:

    def __init__(self, action_name, path_to_action, robot):
        self.action_name = action_name
        self.path_to_action = path_to_action
        self.robot = robot
        self.graph_model = gm.GraphModel.load(self.path_to_action)

    #def _state_machine_status_cb(self):

    def execute(self, actserver):
        r = rospy.Rate(30)
        #self.graph_model.register_status_cb(self._state_machine_status_cb)
        state_machine = self.graph_model.create_state_machine(self.robot)
        self.graph_model.run(self.action_name, state_machine=state_machine)
        state_machine_output = None

        #Check on execution status
        while True:
            if actserver.is_preempt_requested():
                self.graph_model.preempt()
                actserver.set_preempted()
                success = False
                break

            if not self.graph_model.is_running():
                outcome = self.graph_model.get_last_outcome()
                if outcome != None:
                    state_machine_output, end_time = outcome
                    success = True
                else:
                    success = False
                break

            r.sleep()

        if success:
            result = RunScriptResult(state_machine_output)
            rospy.loginfo("%s: succeeded with %s" % (self.action_name, state_machine_output))
            actserver.set_succeeded(result)
        else:
            actserver.set_aborted()

def run(robot, path):
#    from optparse import OptionParser
    #p = OptionParser()
    #options, args = p.parse_args()
    #print options.name, options.dir
    app = QtGui.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    server = RCommanderAutoServer(robot, path)
    rospy.loginfo('RCommander server UP!')
    app.exec_()

