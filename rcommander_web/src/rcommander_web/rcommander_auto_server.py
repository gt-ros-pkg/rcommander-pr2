import roslib; roslib.load_manifest('rcommander_web')
import rospy
import actionlib

from rcommander_web.srv import ActionInfo, ActionInfoResponse
from rcommander_web.msg import *
import rcommander.graph_model as gm

from PyQt4 import QtCore#, QtGui
import sys
import signal
import os
import os.path as pt
import numpy as np
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

def has_graph_files(p):
    epath = os.path.join(p, 'edges.graph')
    npath = os.path.join(p, 'nodes.graph')
    return os.path.isfile(epath) and os.path.isfile(npath)



##
# Find folders with graph files in a directory tree return a recursive dict of the form:
# {'path':   just folder name, not full path
#  'actions': [{another folder's dict}, {another folder's dict 2}, string_of_action_file_name}
def find_all_actions(path):
    path = os.path.normpath(path)
    actions = []
    for d in os.listdir(path):
        candidate_dir = os.path.join(path, d)
        if os.path.isdir(candidate_dir) and has_graph_files(candidate_dir):
            actions.append(candidate_dir)
        elif os.path.isdir(candidate_dir):
            sub_dir_candidates = find_all_actions(candidate_dir)
            if len(sub_dir_candidates['actions']) > 0:
                actions.append(sub_dir_candidates)

    return {'path': os.path.split(path)[1], 'actions': actions}


class RCommanderAutoServer:

    def __init__(self, robot, path_to_rcommander_files):
        self.path_to_rcommander_files = path_to_rcommander_files
        self.robot = robot
        self.action_dict = {}
        self.actions_tree = {'path': path_to_rcommander_files, 'actions':[]}
        self.main_dir_watcher = WatchDirectory(self.path_to_rcommander_files, self.main_directory_changed)
        self.main_directory_changed(self.path_to_rcommander_files)

        rospy.Service('list_rcommander_actions', ActionInfo, self.list_action_cb)
        self.actserv = actionlib.SimpleActionServer('run_rcommander_action_web', RunScriptAction, execute_cb=self.execute_cb, auto_start=False)
        self.actserv.start()

    def execute_cb(self, goal):
        rospy.loginfo('Executing: ' + goal.action_path)
        self.action_dict[goal.action_path]['server'].execute(self.actserv)

    #def _find_all_actions(self, path):
    #    path = os.path.normpath(path)
    #    actions = []
    #    for d in os.listdir(path):
    #        candidate_dir = os.path.join(path, d)
    #        if os.path.isdir(candidate_dir) and has_graph_files(candidate_dir):
    #            actions.append(candidate_dir)
    #        elif os.path.isdir(candidate_dir):
    #            sub_dir_candidates = self._find_all_actions(candidate_dir)
    #            if len(sub_dir_candidates['actions']) > 0:
    #                actions.append(sub_dir_candidates)

    #    return {'path': os.path.split(path)[1], 'actions': actions}

    def _load(self, action):
        rospy.loginfo('Loading ' + action)
        #action_path = os.path.join(self.path_to_rcommander_files, action)
        action_path = action
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

    def _create_action_dict(self, actions_tree):
        loaded_actions = {}
        pruned_tree = {'path': actions_tree['path'], 'actions':[]}

        for action in actions_tree['actions']:
            if isinstance(action, dict):
                sub_loaded_actions, sub_pruned_tree = self._create_action_dict(action)
                if len(sub_loaded_actions.keys()) > 0:
                    for k in sub_loaded_actions.keys():
                        loaded_actions[k] = sub_loaded_actions[k]
                    pruned_tree['actions'].append(sub_pruned_tree)
            else:
                if self.action_dict.has_key(action):
                    loaded_actions[action] = self.action_dict[action]
                    pruned_tree['actions'].append(action)
                else:
                    loaded_action = self._load(action)
                    if loaded_action != None:
                        loaded_actions[action] = loaded_action
                        pruned_tree['actions'].append(action)
                    else:
                        rospy.loginfo('Failed to load ' + action)
        return loaded_actions, pruned_tree

    def main_directory_changed(self, main_path_name):
        rospy.loginfo('main_directory_changed: rescanning ' + main_path_name)
        actions = find_all_actions(self.path_to_rcommander_files)
        self.action_dict, self.actions_tree = self._create_action_dict(actions)
        rospy.loginfo('All actions found\n %s \n' % str(self.action_dict.keys()))

    def get_actions(self, path, actions_tree):
        path = os.path.normpath(path)
        #print 'get_actions:', path
        fnames, fpaths = [], []
        anames, apaths = [], []

        #rint 'path before', path
        splitted = path.split(os.path.sep)
        #rint 'path after', path
        #print actions_tree
        #print actions_tree['path'], splitted[0], len(splitted), splitted
        if actions_tree['path'] == splitted[0]:
            #rint 'here1', path
            if len(splitted) == 1:
                #rint 'here2', path
                for a in actions_tree['actions']:
                    if isinstance(a, dict):
                        fnames.append(a['path'])
                        #print 'joining', path, 'with', a['path']
                        fpaths.append(os.path.join(path, a['path']))
                    else:
                        anames.append(os.path.split(a)[1])
                        apaths.append(a)
                #rint 'returned 1', path, '\n'
                return fnames, fpaths, anames, apaths
            else:
                for a in actions_tree['actions']:
                    if isinstance(a, dict) and a['path'] == splitted[1]:
                        #print 'returned 2', '\n'
                        fnames, fpaths, anames, apaths = self.get_actions(os.path.join(*splitted[1:]), a)
                        fpaths = [os.path.join(splitted[0], fn) for fn in fpaths]
                        return fnames, fpaths, anames, apaths 

        #rint 'returned 3', '\n'
        return fnames, fpaths, anames, apaths

    def list_action_cb(self, req):
        path = req.path
        if path == '.':
            path = self.path_to_rcommander_files

        fnames, fpaths, anames, apaths = self.get_actions(path, self.actions_tree)
        forder = np.argsort(fnames)
        fnames = np.array(fnames)[forder].tolist()
        fpaths = np.array(fpaths)[forder].tolist()
        
        aorder = np.argsort(anames)
        anames = np.array(anames)[aorder].tolist()
        apaths = np.array(apaths)[aorder].tolist()

        rospy.loginfo('responded to %s' % path)
        return ActionInfoResponse(fnames, fpaths, anames, apaths)

class ScriptedActionServer:

    def __init__(self, action_name, path_to_action, robot):
        self.action_name = action_name
        self.path_to_action = path_to_action
        self.robot = robot
        self.graph_model = gm.GraphModel.load(self.path_to_action)
        self.last_msg = ''
        self.message = ''

    def _status_cb(self, active_states):
        self.message = active_states[0]

    #def _state_machine_status_cb(self):
    def execute(self, actserver):
        self.last_msg = ''
        r = rospy.Rate(30)
        #self.graph_model.register_status_cb(self._state_machine_status_cb)
        state_machine = self.graph_model.create_state_machine(self.robot)
        self.graph_model.register_start_cb(self._status_cb)
        self.graph_model.register_transition_cb(self._status_cb)
        self.graph_model.run(self.action_name, state_machine=state_machine)
        state_machine_output = None

        #Check on execution status
        while not rospy.is_shutdown():
            if actserver.is_preempt_requested():
                self.graph_model.preempt()
                actserver.set_preempted()
                success = False
                rospy.loginfo('PREEMPTED')
                break

            if not self.graph_model.is_running():
                outcome = self.graph_model.get_last_outcome()
                if outcome != None:
                    state_machine_output, end_time = outcome
                    success = True
                else:
                    success = False
                break

            if self.message != self.last_msg:
                feedback = RunScriptFeedback('node', self.message)
                self.last_msg = self.message
                actserver.publish_feedback(feedback)
                #print "ASDFE TESTING. PUBLISHING FEEDBACK", self.message

            r.sleep()

        if success:
            result = RunScriptResult(state_machine_output)
            rospy.loginfo("%s: succeeded with %s" % (self.action_name, state_machine_output))
            actserver.set_succeeded(result)
        else:
            rospy.loginfo('Aborted')
            actserver.set_aborted()

def run(robot, path):
#    from optparse import OptionParser
    #p = OptionParser()
    #options, args = p.parse_args()
    #print options.name, options.dir
    #app = QtGui.QApplication(sys.argv)
    app = QtCore.QCoreApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    server = RCommanderAutoServer(robot, path)
    rospy.loginfo('RCommander server UP!')
    app.exec_()

##
# list_rcommander_actions(full path)
#   break full path up, select the first of path, descend until end then return dict element

# list_rcommander_actions('.')
# list_rcommander_actions('path')
# execute(path) <==> provide feedback and such
# 

