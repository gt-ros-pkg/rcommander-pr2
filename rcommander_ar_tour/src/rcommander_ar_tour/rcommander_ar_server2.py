import roslib; roslib.load_manifest("rcommander_ar_tour")
import rospy
import actionlib
import std_msgs.msg as stdm

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh
import ar_pose.msg as ar_msg
from rcommander_web.srv import ActionInfo, ActionInfoResponse
import geometry_msgs.msg as gmsg

import rcommander_web.msg as rmsg
#from rcommander_web.msg import *
#import rcommander_ar_pose.utils as rap
import rcommander_pr2_gui.tf_utils as tfu
#import rcommander.graph_model as gm

from PyQt4 import QtCore#, QtGui
#import sys
#import signal
import os
import os.path as pt
import numpy as np
import inspect as insp
#from inspect import isfunction


import rcommander_web.rcommander_auto_server as ras
import tf
import cPickle as pk
import functools as ft

import re
import copy
from threading import RLock
import pdb

DEFAULT_LOC = [[0.,0.,0.], [0.,0.,0.,1.]]

def interactive_marker(name, pose, scale):
    int_marker = ims.InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.pose.position.x = pose[0][0]
    int_marker.pose.position.y = pose[0][1]
    int_marker.pose.position.z = pose[0][2]
    int_marker.pose.orientation.x = pose[1][0]
    int_marker.pose.orientation.y = pose[1][1]
    int_marker.pose.orientation.z = pose[1][2]
    int_marker.pose.orientation.w = pose[1][2]
    
    int_marker.scale = scale
    int_marker.name = name
    int_marker.description = name
    return int_marker

def make_rviz_marker(scale):
    marker = ims.Marker()
    marker.type = ims.Marker.SPHERE
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color = stdm.ColorRGBA(.5,.5,.5,1)
    return marker


def make_sphere_control(name, scale):
    control =  ims.InteractiveMarkerControl()
    control.name = name + '_sphere'
    control.always_visible = True
    control.markers.append(make_rviz_marker(scale))
    control.interaction_mode = ims.InteractiveMarkerControl.BUTTON
    return control

def make_control_marker():
    control = ims.InteractiveMarkerControl()
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 0
    control.orientation.w = 1
    control.interaction_mode = ims.InteractiveMarkerControl.MOVE_AXIS
    return control

def make_directional_controls(name):
    x_control = make_control_marker()
    x_control.orientation.x = 1
    x_control.name = name + "_move_x"

    y_control = make_control_marker()
    y_control.orientation.y = 1
    y_control.name = name + "_move_y"

    z_control = make_control_marker()
    z_control.orientation.z = 1
    z_control.name = name + "_move_z"

    return [x_control, y_control, z_control]

def pose_to_tup(p):
    return [p.position.x, p.position.y, p.position.z], \
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

##
# @param callback is a function: f(menu_item, full_action_path)
def menu_handler_from_action_tree(actions_tree, callback):
    handler = mh.MenuHandler()
    menu_handler_from_action_tree_helper(handler, actions_tree, callback)
    return handler

# {'path':   full path
#  'actions': [{another folder}, {another folder2}, rcom_file}
def menu_handler_from_action_tree_helper(handler, actions_tree, callback, parent=None):
    base_path = actions_tree['path']
    for action in actions_tree['actions']:
        if isinstance(action, dict):
            submenu = handler.insert(action['path'], parent=parent)
            menu_handler_from_action_tree_helper(handler, action, callback, parent=submenu)
        else:
            action_name = os.path.split(action)[1]
            handler.insert(action_name, parent=parent, callback=ft.partial(callback, action_name, action))

def Database_load(name, db_class):
    try:
        pickle_file = open(name, 'r')
        db = pk.load(pickle_file)
        pickle_file.close()
        action_db = db_class()
        action_db.database = db
        action_db.modified = False
        rospy.loginfo('Loaded %s.' % name)
        return action_db
    except Exception, e:
        rospy.loginfo('Error loading %s. Creating new database.' % name)
        return db_class()

#return a tup (pos, orientation)
def lookup_transform(listener, aframe, bframe)
    try:
        return listener.lookupTransform(aframe, bframe, rospy.Time(0)) 

    except tf.ExtrapolationException, e:
        print 'exception', e
        return None

    except tf.ConnectivityException, e:
        print 'exception', e
        return None

    except tf.LookupException, e:
        print 'exception', e.__class__
        return None


class Database:

    def __init__(self):
       self.modified = True
       self.database = {}

    def get(self, an_id):
        return self.database[an_id].copy()

    def has_id(self, an_id):
        return an_id in self.database.keys()

    def ids(self):
        return self.database.keys()

    def save(self, name):
        if self.modified:
            self.modified = False
            pickle_file = open(name, 'w')
            pk.dump(self.database, pickle_file)
            pickle_file.close()
            rospy.loginfo('Saved actions database to %s.'% name)

    def remove(self, an_id):
        self.database.pop(an_id)
        self.modified = True


class ActionDatabase(Database):
       
   def __init__(self):
        Database.__init__(self)

   def insert(self, name, frame, action_pose=DEFAULT_LOC, behavior_path=None):
       action_id = name + str(rospy.get_rostime().to_time())
       self.database[action_id] = {'name': name, 'frame': frame, 
                                   'loc': action_pose, 'behavior_path': behavior_path}
       self.modified = True
       return action_id

    def update_frame(self, action_id, frame):
        self.database[action_id]['frame'] = frame
        self.modified = True

    def update_behavior(self, action_id, behavior):
        self.database[action_id]['behavior'] = behavior
        self.modified = True

    def update_loc(self, action_id, loc):
        self.database[action_id]['loc'] = loc

class ActionMarker:

    ##
    # the marker represents itself in the /map frame internally but presents
    # an interface in its defined frame
    def __init__(self, manager, actionid, location, frame, marker_server, server_lock ,tf_listener, menu_handler=None):
        self.manager = manager
        self.actionid = actionid
        self.frame = frame
        self.location = location
        self.marker_server = marker_server
        self.server_lock = server_lock
        self.tf_listener = tf_listener

        self.marker_obj = None
        self.marker_name = None
        self.menu_handler = menu_handler
        self.is_current_task_frame = False
        self._make_marker()

    def set_selected(self, b):
        self.is_current_task_frame = b
        if b == self.is_current_task_frame:
            return

        if self.marker_obj != None:
            self.remove()
            self._make_marker()

    def remove(self):
        if self.marker_obj != None:
            self.server_lock.acquire()
            self.marker_server.erase(self.marker_name)
            self.server_lock.release()

            self.marker_obj = None
            self.marker_name = None

    def update(self, pose_in_defined_frame):
        pose = gmsg.Pose()
        pose.position.x = pose_in_defined_frame[0][0]
        pose.position.y = pose_in_defined_frame[0][1]
        pose.position.z = pose_in_defined_frame[0][2]
        pose.orientation.x = pose_in_defined_frame[1][0]
        pose.orientation.y = pose_in_defined_frame[1][1]
        pose.orientation.z = pose_in_defined_frame[1][2]
        pose.orientation.w = pose_in_defined_frame[1][3]

        self.server_lock.acquire()
        self.marker_server.setPose(self.marker_name, pose)
        self.server_lock.release()

    def _make_marker(self): #, scale=.2, color=stdm.ColorRGBA(.5,.5,.5,.5)):
        if self.is_current_task_frame:
            scale = .3
            color = stdm.ColorRGBA(1,0,0,.5)
        else:
            scale = .2
            color = stdm.ColorRGBA(.5,.5,.5,.5)

        self.marker_name = 'action_' + self.actionid
        p_ar = self.location #self.tag_database.get(self.tagid)['target_location']
        m_ar = tfu.tf_as_matrix(p_ar)
        map_T_ar = tfu.transform('map', self.frame, self.tf_listener, t=0)
        p_map = tfu.matrix_as_tf(map_T_ar * m_ar)
        pose = p_map

        int_marker = interactive_marker(self.marker_name, (pose[0], (0,0,0,1)), scale)
        int_marker.header.frame_id = 'map' #self.tagid
        int_marker.description = ''
        int_marker.controls.append(make_sphere_control(self.marker_name, scale/2.))
        int_marker.controls[0].markers[0].color = color
        int_marker.controls += make_directional_controls(self.marker_name)

        self.server_lock.acquire()
        self.marker_server.insert(int_marker, self.marker_cb)
        self.server_lock.release()
        self.marker_obj = int_marker
        if self.menu_handler != None:
            self._make_menu()

    def _make_menu(self):
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = self.actionid + '_menu'
        menu_control.markers.append(copy.deepcopy(self.marker_obj.controls[0].markers[0]))
        menu_control.always_visible = True
        self.marker_obj.controls.append(copy.deepcopy(menu_control))

        self.server_lock.acquire()
        self.marker_server.insert(self.marker_obj, self.marker_cb)
        self.menu_handler.apply(self.marker_server, self.marker_obj.name)
        self.server_lock.release()

    def set_menu(self, menu_handler):
        self.menu_handler = menu_handler

        #erase current marker
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        self.server_lock.release()

        #insert it with a menu_handler
        self._make_menu()

    def marker_cb(self, feedback):
        print 'called', feedback_to_string(feedback.event_type), feedback.marker_name, feedback.control_name
        if feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            p_map = tfu.tf_as_matrix(pose_to_tup(feedback.pose))
            ar_T_map = tfu.transform(self.tagid, 'map', self.tf_listener, t=0)
            p_ar = tfu.matrix_as_tf(ar_T_map * p_map)
            self.manager.update_loc(self.actionid, p_ar)

        sphere_match = re.search('_sphere$', feedback.control_name)
        menu_match = re.search('_menu$', feedback.control_name)
        if ((sphere_match != None) or (menu_match != None)) and target_match != None \
                and feedback.event_type == ims.InteractiveMarkerFeedback.MOUSE_DOWN:
            self.manager.selected_cb(self.actionid)
            #self.frame_selected_cb(self.actionid)


class ActionMarkersManager:

    def __init__(self, action_marker_database_name, server_lock, behavior_server, marker_server, tf_listener):
        self.server_lock = server_lock
        self.marker_db = Database_load(action_marker_database_name, ActionDatabase)
        self.behavior_server = behavior_server
        self.marker_server = marker_server
        self.tf_listener = tf_listener

        self.markers = {}
        self.server_lock.acquire()
        for action_id in self.marker_db.ids():
            rec = self.marker_db.get(action_id)
            self._create_marker(action_id, rec['loc'], rec['frame'])
        self.marker_server.applyChanges()
        self.server_lock.release()

    def create_action(self, frame):
        actionid = self.marker_db.insert('action', frame)
        rec = self.marker_db.get(actionid)
        self._create_marker(actionid, rec['loc'], rec['frame'])
    
    # create the interactive marker obj
    def _create_marker(self, actionid, location, frame):
        self.markers[actionid] = ActionMarker(self, actionid, location, frame, 
                self.marker_server, self.server_lock, self.tf_listener,
                self._create_menu_handler(actionid))

    def _create_menu_handler(self, actionid):
        return menu_handler_from_action_tree(self.behavior_server.get_actions_tree(), 
                    ft.partial(self._action_selection_menu_cb, actionid))

    # callback from ARTagMarker to perform selection
    def selected_cb(self, actionid):
        for k in self.markers.keys():
            self.markers[k].set_selected(k == actionid)

    # callback from ARTagMarker to update location of action
    def update_loc(self, actionid, loc):
        self.marker_db.update_loc(actionid, loc)

    def update_behavior_menus(self):
        self.ar_lock.acquire()
        for actionid in self.markers.keys():
            menu_handler = self._create_menu_handler(actionid)
            self.markers[actionid].set_menu(menu_handler)
        self.marker_server.applyChanges()
        self.ar_lock.release()

    def _action_selection_menu_cb(self, actionid, menu_item, full_path, int_feedback):
        print 'menu called back on', actionid, menu_item, full_path #, int_feedback
        #self.ar_lock.acquire()
        #print 'got ar lock'

        #marker_disp = self.marker_displays_dict[actionid]
        #self.tag_database.update_behavior(marker_disp.action_id, full_action_path)
        #self._insert_database_action_into_actions_tree(marker_disp.action_id)
        #marker_disp.update_ar_marker(menu_item)
        #self.marker_server.applyChanges()

        #self.ar_lock.release()
        #print 'ar lock released'


    #def create_menu_for_all_markers(self, actions_tree):
    #    #Create the menu object on all markers
    #    self.ar_lock.acquire()
    #    for marker_name in self.marker_displays_dict.keys():
    #        menu_handler = menu_handler_from_action_tree(actions_tree, ft.partial(self.action_selection_menu_cb, marker_name))
    #        self.marker_displays_dict[marker_name].set_menu(menu_handler)
    #    self.marker_server.applyChanges()
    #    self.ar_lock.release()


class ARTagDatabase(Database):

    def __init__(self):
        Database.__init__(self)

    def set_location(self, tag_id, tag_location): 
        self.database[tag_id] = tag_location 
        self.modified = True
        return tag_id

##
# User can create multiple different action markers from ar tag markers
class ARTagMarker:

    def __init__(self, tagid, pose_in_map_frame, action_manager, server_lock, marker_server, scale=.2):
        self.tagid = tagid
        self.action_manager = action_manager
        self.server_lock = server_lock
        self.marker_server = marker_server
        self.marker_name = 'ar_marker_' + str(self.tagid)

        int_marker = interactive_marker(self.marker_name, pose_in_map_frame, scale)
        self.marker_obj = int_marker
        int_marker.description = 'Tag #%d' % self.tagid
        int_marker.controls += [make_sphere_control(name, int_marker.scale)]
        int_marker.controls[0].markers[0].color = stdm.ColorRGBA(0,1,0,.5)


        menu = mh.MenuHandler()
        menu.insert('Create New Action', parent=None, callback=self.create_new_action_cb)
        self.server_lock.acquire()
        self.marker_server.insert(int_marker, self.marker_cb)
        self.server_lock.release()

    def create_new_action_cb(self, feedback):
        self.action_manager.create_action(self.tagid)

    def marker_cb(self, feedback):
        print 'ARTagMarker: clicked on', feedback.marker_name

    def update(self, pose_in_map_frame):
        pose = gmsg.Pose()
        pose.position.x = pose_in_map_frame[0][0]
        pose.position.y = pose_in_map_frame[0][1]
        pose.position.z = pose_in_map_frame[0][2]
        pose.orientation.x = pose_in_map_frame[1][0]
        pose.orientation.y = pose_in_map_frame[1][1]
        pose.orientation.z = pose_in_map_frame[1][2]
        pose.orientation.w = pose_in_map_frame[1][3]

        self.server_lock.acquire()
        self.marker_server.setPose(self.marker_name, pose)
        self.server_lock.release()

    def remove(self):
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        self.server_lock.release()


class ARMarkersManager:

    def __init__(self, ar_tag_database_name, server_lock, marker_server, 
                tf_listener, tf_broadcaster):
        self.server_lock = server_lock
        self.marker_db = Database_load(ar_tag_database_name, ARTagDatabase)
        self.marker_server = marker_server
        self.tf_listener = tf_listener
        self.broadcaster = broadcaster

        self.markers = {}
        self.white_list = []

        self.server_lock.acquire()
        for mid in self.marker_db.ids():
            self.create_marker(mid, self.marker_db.get(mid))
        self.marker_server.applyChanges()
        self.server_lock.release()

    ##
    # set ids of tags that should not be deleted when they are no longer visible
    def set_white_list_ids(self, white_list):
        self.white_list = white_list
        
    def _frame_name(self, tagid):
        return '4x4_' + tagid

    def create_marker(self, tagid, location):
        self.markers[tagid] = ARTagMarker(tagid, location, server_lock=self.server_lock, 
                                            marker_server=self.marker_server)

    def update(self, visible_markers):
        #visible_markers = {}
        #for marker in msg.markers:
        #    visible_markers['4x4_' + str(marker.id)] = marker.id
        #self.visible_markers = visible_markers

        visible_ids = [m.id for m in visible_markers]
        for mid in visible_ids:
            fn = self._frame_name(mid)
            ar_location = lookup_transform(self.tf_listener, 'map', fn)
            if ar_location != None:
                if not self.markers.has_key(mid):
                    self.create_marker(mid, ar_location)
                    self.markers[mid].update(ar_location)
                self.marker_db.set_location(mid, ar_location)

        for mid in self.markers.keys():
            if mid in visible_ids:
                continue
            if mid in self.white_list:
                position, orientation = self.marker_db.get(mid)['ar_location']
                self.broadcaster.sendTransform(position, orientation, rospy.Time.now(), 
                            self._frame_name(mid), '/map')
            else:
                self.marker_db.remove(mid)
                self.markers[mid].remove()
                self.markers.pop(mid)


class BehaviorServer(self):

    def __init__(self, action_database_name, ar_tag_database_name, 
                    path_to_rcommander_files, tf_listener, robot):

        self.action_database_name = action_database_name
        self.ar_tag_database_name = ar_tag_database_name
        self.path_to_rcommander_files = path_to_rcommander_files
        self.robot = robot

        self.broadcaster = tf.TransformBroadcaster()
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.ar_lock = RLock()
        self.loaded_actions = {}

        self.SERVER_NAME = 'behavior_server'
        self.marker_server = None
        self.ar_marker_manager = None
        self.action_marker_manager = None
        self.ar_pose_sub = None
        self.visible_markers = {}
        self.loaded_actions = None
        self.actions_tree = None

        self.create_actions_tree()
        self.start_marker_server()

    def get_actions_tree(self):
        return self.actions_tree

    def start_marker_server(self):
        self.marker_server = ims.InteractiveMarkerServer(self.SERVER_NAME)

        self.ar_marker_manager = ARMarkersManager(self.ar_tag_database_name, self.ar_lock, 
                self.marker_server, self.tf_listener, self.broadcaster)

        self.action_marker_manager = ActionMarkersManager(self.action_tag_database_name, self.ar_lock, 
                self, self.marker_server, self.tf_listener, self.broadcaster)
        self.action_marker_manager.update_behavior_menus()

        #Subscribe to AR Pose to determine marker visibility
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", ar_msg.ARMarkers, self.ar_marker_cb)
        rospy.loginfo('Ready!')

    def ar_marker_cb(self, msg):
        self.visible_markers = msg.markers

    ##
    # Using the recursive dictionary listing in actions_tree goes through and
    # load each action found into memory.  creates an action tree with the same
    # format as its inputs but with actions that don't load removed.
    def load_action_from_found_paths(self, actions_tree):
        loaded_actions = {}
        pruned_tree = {'path': actions_tree['path'], 'actions':[]}

        for action in actions_tree['actions']:
            if isinstance(action, dict):
                sub_loaded_actions, sub_pruned_tree = self.load_action_from_found_paths(action)
                if len(sub_loaded_actions.keys()) > 0:
                    for k in sub_loaded_actions.keys():
                        loaded_actions[k] = sub_loaded_actions[k]
                    pruned_tree['actions'].append(sub_pruned_tree)
            else:
                if self.loaded_actions.has_key(action):
                    loaded_actions[action] = self.loaded_actions[action]
                    pruned_tree['actions'].append(action)
                else:
                    loaded_action = self._load_action_at_path(action)
                    if loaded_action != None:
                        loaded_actions[action] = loaded_action
                        pruned_tree['actions'].append(action)
                    else:
                        rospy.loginfo('Failed to load ' + action)

        return loaded_actions, pruned_tree


    def create_actions_tree(self):
        rospy.loginfo('create_actions_tree: rescanning ' + self.path_to_rcommander_files)
        actions = ras.find_all_actions(self.path_to_rcommander_files)
        self.loaded_actions, self.actions_tree = self.load_action_from_found_paths(actions)
        #self.insert_locations_folder()
        #self.insert_database_actions()

        rospy.loginfo('All actions found\n')
        for k in self.loaded_actions.keys():
            rospy.loginfo('ACTION %s' % k)

    def step(self, timer_obj):
        self.ar_marker_manager.update(self.visible_markers)

    def start(self):
        rospy.Timer(rospy.Duration(.1), self.step)


def run(robot, tf_listener, action_database_name, ar_tag_database_name, path_to_rcommander_files):
    import sys
    from PyQt4 import QtCore#, QtGui
    import signal

    app = QtCore.QCoreApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    behavior_server = BehaviorServer(action_database_name, ar_tag_database_name, 
            path_to_rcommander_files, tf_listener, robot)
    behavior_server.start()

    rospy.loginfo('RCommander AR Tour Server up!')
    app.exec_()














