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
import pypr2.tf_utils as tfu
#import rcommander.graph_model as gm

from PyQt4 import QtCore#, QtGui
#import sys
#import signal
import os
import os.path as pt
import numpy as np
import inspect as insp
#from inspect import isfunction

import detect_robot_move as drm
import rcommander_web.rcommander_auto_server as ras
import tf
import cPickle as pk
import functools as ft

import re
import copy
from threading import RLock
import pdb

DEFAULT_LOC = [[0.,0.,0.], [0.,0.,0.,1.]]

def tag_name(tagid, behavior_name):
    return '%s (Tag #%d)' % (behavior_name, tagid)

def feedback_to_string(ftype):
    names = ['keep_alive', 'pose_update', 
             'menu_select', 'button_click',
             'mouse_down', 'mouse_up']
    fb = ims.InteractiveMarkerFeedback
    consts = [fb.KEEP_ALIVE, fb.POSE_UPDATE,
                fb.MENU_SELECT, fb.BUTTON_CLICK,
                fb.MOUSE_DOWN, fb.MOUSE_UP]

    for n, value in zip(names, consts):
        if ftype == value:
            return n

    return 'invalid type'


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
def lookup_transform(listener, aframe, bframe):
    try:
        return listener.lookupTransform(aframe, bframe, rospy.Time(0)) 

    except tf.ExtrapolationException, e:
        rospy.logdebug( 'exception %s' % str(e))
        return None

    except tf.ConnectivityException, e:
        rospy.logdebug( 'exception %s' % str(e))
        return None

    except tf.LookupException, e:
        rospy.logdebug( 'exception %s' % str(e.__class__))
        return None

#def tfu_transform(aframe, bframe, listener):
#    map_T_ar = tfu.transform(aframe, bframe, self.tf_listener, t=0)

def find_folder(actions_tree, folder_name):
    return find_folder_idx(actions_tree, folder_name)[0]

def find_folder_idx(actions_tree, folder_name):
    for idx, action in enumerate(actions_tree['actions']):
        if isinstance(action, dict) and action['path'] == folder_name:
            return action, idx
    return None, None

class Database:

    def __init__(self):
       self.modified = True
       self.database = {}

    def get(self, an_id):
        return copy.deepcopy(self.database[an_id])

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
            #rospy.loginfo('Saved actions database to %s.'% name)

    def remove(self, an_id):
        self.database.pop(an_id)
        self.modified = True


class ActionDatabase(Database):
       
   def __init__(self):
        Database.__init__(self)

   def insert(self, name, frame, tagid, action_pose=DEFAULT_LOC, behavior_path=None):
       actionid = name + str(rospy.get_rostime().to_time())
       self.database[actionid] = {'name': name, 'frame': frame, 
                                   'loc': action_pose, 'behavior_path': behavior_path,
                                   'tagid': tagid}
       self.modified = True
       return actionid

   def update_frame(self, actionid, frame):
       self.database[actionid]['frame'] = frame
       self.modified = True

   def update_behavior(self, actionid, behavior):
       self.database[actionid]['behavior_path'] = behavior
       self.modified = True

   def update_loc(self, actionid, loc):
       self.database[actionid]['loc'] = loc
       self.modified = True

class ActionMarker:

    ##
    # the marker represents itself in the /map frame internally but presents
    # an interface in its defined frame
    def __init__(self, manager, actionid, location_in_frame, frame, tagid, behavior_name, 
            marker_server, server_lock, tf_listener, menu_handler=None):
        self.manager = manager
        self.actionid = actionid
        self.frame = frame
        self.tagid = tagid
        self.behavior_name = behavior_name
        self.location_in_frame = location_in_frame
        self.marker_server = marker_server
        self.server_lock = server_lock
        self.tf_listener = tf_listener

        self.marker_obj = None
        self.marker_name = None
        self.menu_handler = menu_handler
        if menu_handler != None:
            self._add_delete_option(menu_handler)
        else:
            self._make_empty_menu()
        self.is_current_task_frame = False
        self._make_marker()

    def _add_delete_option(self, menu_handler):
        menu_handler.insert('-----------------', parent=None, callback=None)
        menu_handler.insert('Delete', parent=None, callback=self.delete_action_cb)

    def delete_action_cb(self, feedback):
        self.manager.delete_marker_cb(self.actionid)

    def set_selected(self, b):
        old_status = self.is_current_task_frame
        if old_status == True and b == True:
            b = False

        self.is_current_task_frame = b
        if b == old_status:
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
        self.marker_obj.pose = pose_in_defined_frame
        self.location_in_frame = pose_to_tup(pose_in_defined_frame)

    def _to_map_frame(self, p_ar):
        m_ar = tfu.tf_as_matrix(p_ar)
        self.tf_listener.waitForTransform('map', self.frame, rospy.Time.now(), rospy.Duration(10))
        map_T_ar = tfu.transform('map', self.frame, self.tf_listener, t=0)
        p_map = tfu.matrix_as_tf(map_T_ar * m_ar)
        return p_map

    def _make_marker(self): #, scale=.2, color=stdm.ColorRGBA(.5,.5,.5,.5)):
        if self.is_current_task_frame:
            scale = .3
            color = stdm.ColorRGBA(1,0,0,.4)
        else:
            scale = .2
            color = stdm.ColorRGBA(.5,.5,.5,.4)

        self.marker_name = 'action_' + self.actionid
        pose = self.location_in_frame

        int_marker = interactive_marker(self.marker_name, (pose[0], (0,0,0,1)), scale)
        int_marker.header.frame_id = self.frame #self.tagid
        int_marker.description = self._make_description()
        int_marker.controls.append(make_sphere_control(self.marker_name + '_1', scale/8.))
        int_marker.controls.append(make_sphere_control(self.marker_name + '_2', scale))
        int_marker.controls[1].markers[0].color = color
        int_marker.controls += make_directional_controls(self.marker_name)

        self.server_lock.acquire()
        self.marker_server.insert(int_marker, self.marker_cb)
        self.server_lock.release()
        self.marker_obj = int_marker
        if self.menu_handler != None:
            self._make_menu()
        else:
            self._make_empty_menu()

    def _make_empty_menu(self):
        self.menu_handler = mh.MenuHandler()
        self._add_delete_option(self.menu_handler)
        self._make_menu()

    def _make_menu(self):
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = 'menu_' + self.actionid
        menu_control.markers.append(copy.deepcopy(self.marker_obj.controls[0].markers[0]))
        menu_control.always_visible = True
        self.marker_obj.controls.append(copy.deepcopy(menu_control))

        self.server_lock.acquire()
        self.marker_server.insert(self.marker_obj, self.marker_cb)
        self.menu_handler.apply(self.marker_server, self.marker_obj.name)
        self.server_lock.release()

    def set_menu(self, menu_handler):
        self._add_delete_option(menu_handler)
        self.menu_handler = menu_handler

        #erase current marker
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        self.server_lock.release()

        #insert it with a menu_handler
        self._make_menu()

    def _make_description(self):
        if self.behavior_name != None:
            return tag_name(self.tagid, self.behavior_name)
            #return '%s (Tag %d)' % (self.behavior_name, self.tagid)
        else:
            return ''

    def update_name(self, behavior_name):
        self.behavior_name = behavior_name
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        
        self.marker_obj.description = self._make_description()
        self.marker_server.insert(self.marker_obj, self.marker_cb)
        self.marker_server.applyChanges()
        self.server_lock.release()

    def marker_cb(self, feedback):
        #print 'called', feedback_to_string(feedback.event_type), feedback.marker_name, feedback.control_name
        if feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            #p_ar = tfu.tf_as_matrix(pose_to_tup(feedback.pose))
            p_ar = pose_to_tup(feedback.pose)
            #self.tf_listener.waitForTransform(self.frame, 'map', 
            #        rospy.Time.now(), rospy.Duration(10))
            #ar_T_map = tfu.transform(self.frame, 'map', self.tf_listener, t=0)
            #p_ar = tfu.matrix_as_tf(ar_T_map * p_map)
            self.manager.update_loc(self.actionid, p_ar)
            self.update(feedback.pose)

        sphere_match = re.search('_sphere$', feedback.control_name)
        if (sphere_match != None) \
                and feedback.event_type == ims.InteractiveMarkerFeedback.MOUSE_DOWN:
            self.manager.set_task_frame(self.actionid)
            self.server_lock.acquire()
            self.marker_server.applyChanges()
            self.server_lock.release()
            #self.frame_selected_cb(self.actionid)


class ActionMarkersManager:

    def __init__(self, action_marker_database_name, server_lock, behavior_server, 
                 marker_server, actions_db_changed_cb, tf_listener):
        self.server_lock = server_lock
        self.behavior_server = behavior_server
        self.marker_server = marker_server
        self.actions_db_changed_cb = actions_db_changed_cb
        self.tf_listener = tf_listener

        #Database and marker list has to be in sync!
        self.marker_db = Database_load(action_marker_database_name, ActionDatabase)
        self.markers = {}

        self.server_lock.acquire()
        for action_id in self.marker_db.ids():
            self._create_marker(action_id)
        self.marker_server.applyChanges()
        self.server_lock.release()

    def get_current_task_frame(self):
        for actionid in self.markers.keys():
            marker = self.markers[actionid]
            if marker.is_current_task_frame:
                return marker.frame, marker.location_in_frame
        return None

    def _get_behavior_name(self, rec):
        if rec['behavior_path'] != None:
            return pt.split(rec['behavior_path'])[1]
        else:
            return None

    def create_action(self, frame, tagid):
        #print 'create_action: tagid class', tagid.__class__
        actionid = self.marker_db.insert('action', frame, tagid)
        rec = self.marker_db.get(actionid)
        self._create_marker(actionid)
        self.server_lock.acquire()
        self.marker_server.applyChanges()
        self.server_lock.release()
    
    # create the interactive marker obj
    def _create_marker(self, actionid):#, location_in_frame, frame, behavior_name, tagid):
        rec = self.marker_db.get(actionid)
        self.markers[actionid] = ActionMarker(self, actionid, rec['loc'], 
                rec['frame'], rec['tagid'], self._get_behavior_name(rec), self.marker_server, 
                self.server_lock, self.tf_listener, self._create_menu_handler(actionid))

    def _create_menu_handler(self, actionid):
        tree = self.behavior_server.get_actions_tree()
        if tree == None:
            return None

        behavior_folder = find_folder(tree, 'Behaviors')
        if behavior_folder != None:
            ar_folder = copy.deepcopy(find_folder(behavior_folder, 'AR'))
            if ar_folder != None:
                ar_folder['path'] = 'Behaviors'
                tree = {'path':'root', 'actions':[ar_folder]}

            return menu_handler_from_action_tree(tree, 
                    ft.partial(self._action_selection_menu_cb, actionid))
        else:
            return None

    def delete_marker_cb(self, actionid):
        print 'delete called on', actionid
        self.markers[actionid].remove()
        self.markers.pop(actionid)
        self.marker_db.remove(actionid)
        self.server_lock.acquire()
        self.marker_server.applyChanges()
        self.server_lock.release()
        self.actions_db_changed_cb()

    # callback from ARTagMarker to perform selection
    def set_task_frame(self, actionid):
        for k in self.markers.keys():
            self.markers[k].set_selected(k == actionid)

    # callback from ARTagMarker to update location of action
    def update_loc(self, actionid, loc):
        self.marker_db.update_loc(actionid, loc)

    def update_behavior_menus(self):
        self.server_lock.acquire()
        for actionid in self.markers.keys():
            menu_handler = self._create_menu_handler(actionid)
            self.markers[actionid].set_menu(menu_handler)
        self.marker_server.applyChanges()
        self.server_lock.release()

    def _action_selection_menu_cb(self, actionid, menu_item, full_action_path, int_feedback):
        print 'menu called back on', actionid, menu_item, full_action_path #, int_feedback
        self.marker_db.update_behavior(actionid, full_action_path)
        self.markers[actionid].update_name(menu_item)
        self.actions_db_changed_cb()


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

    def __init__(self, tagid, pose_in_map_frame, action_marker_manager, server_lock, marker_server, scale=.2):
        self.tagid = tagid
        self.action_marker_manager = action_marker_manager
        self.server_lock = server_lock
        self.marker_server = marker_server
        self.marker_name = 'ar_marker_' + str(self.tagid)

        int_marker = interactive_marker(self.marker_name, pose_in_map_frame, scale)
        int_marker.scale = .6
        self.marker_obj = int_marker
        int_marker.description = 'Tag #%d' % self.tagid

        #int_marker.controls += []
        #int_marker.controls[0].markers[0].color = stdm.ColorRGBA(0,1,0,.5)
        sph = make_sphere_control(self.marker_name, scale)
        sph.markers[0].color = stdm.ColorRGBA(0,1,0,.5)

        self.menu = mh.MenuHandler()
        self.menu.insert('Create New Action', parent=None, callback=self.create_new_action_cb)
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = 'menu_tag_' + str(self.tagid)
        menu_control.markers.append(copy.deepcopy(sph.markers[0]))
        menu_control.always_visible = True
        int_marker.controls += [menu_control]

        self.server_lock.acquire()
        self.marker_server.insert(int_marker, self.marker_cb)
        self.menu.apply(self.marker_server, int_marker.name)
        self.server_lock.release()

    def create_new_action_cb(self, feedback):
        #print 'create_new_action_cb', feedback
        rospy.loginfo('New action created! Tag id %d.' % self.tagid)
        self.action_marker_manager.create_action(ar_frame_name(self.tagid), self.tagid)

    def marker_cb(self, feedback):
        pass
        #print 'ARTagMarker: clicked on', feedback.marker_name

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

def ar_frame_name(tagid):
    return '4x4_' + str(tagid)


class ARMarkersManager:

    def __init__(self, ar_tag_database_name, action_marker_manager, server_lock, marker_server, 
                tf_listener, tf_broadcaster):

        self.robot_movement_detector = drm.DetectRobotMove()
        self.marker_db = Database_load(ar_tag_database_name, ARTagDatabase)
        self.action_marker_manager = action_marker_manager
        self.server_lock = server_lock
        self.marker_server = marker_server
        self.tf_listener = tf_listener
        self.broadcaster = tf_broadcaster

        self.markers = {}

        self.server_lock.acquire()
        for mid in self.marker_db.ids():
            self.create_marker(mid, self.marker_db.get(mid))
        self.marker_server.applyChanges()
        self.server_lock.release()

    def create_marker(self, tagid, location):
        self.markers[tagid] = ARTagMarker(tagid, location, self.action_marker_manager, self.server_lock, self.marker_server)

    def remove_marker(self, tagid):
        self.markers[tagid].remove()
        self.markers.pop(tagid)
        self.marker_db.remove(tagid)

    def update(self, visible_markers):
        visible_ids = [m.id for m in visible_markers]
        markers_changed = False

        if not self.robot_movement_detector.is_moving():
            for mid in visible_ids:
                fn = ar_frame_name(mid)
                ar_location = lookup_transform(self.tf_listener, 'map', fn)
                if ar_location != None:
                    if not self.markers.has_key(mid):
                        #print 'UPDATE', mid.__class__
                        self.create_marker(mid, ar_location)
                        self.markers[mid].update(ar_location)
                        markers_changed = True
                    else:
                        self.markers[mid].update(ar_location)
                        markers_changed = True
                    self.marker_db.set_location(mid, ar_location)
#else:
#            rospy.loginfo('skipped updating ARMarkers because we are moving')

        for mid in self.markers.keys():
            if mid in visible_ids:
                continue
            position, orientation = self.marker_db.get(mid)
            self.broadcaster.sendTransform(position, orientation, rospy.Time.now(), 
                        ar_frame_name(mid), '/map')

        return markers_changed


class BehaviorServer:

    def __init__(self, action_tag_database_name, ar_tag_database_name, 
                    path_to_rcommander_files, tf_listener, robot):

        self.action_tag_database_name = action_tag_database_name
        self.ar_tag_database_name = ar_tag_database_name
        self.path_to_rcommander_files = path_to_rcommander_files
        self.robot = robot

        self.broadcaster = tf.TransformBroadcaster()
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.marker_server_lock = RLock()
        self.loaded_actions = {}

        self.SERVER_NAME = 'behavior_server'
        self.marker_server = None
        self.ar_marker_manager = None
        self.action_marker_manager = None
        self.ar_pose_sub = None
        self.visible_markers = {}
        self.actions_tree = {'path': self.path_to_rcommander_files, 'actions':[]}
        self.actserv = None

        self.create_actions_tree()
        self.start_marker_server()
        self.start_list_service()
        self.start_execution_action_server()
        self.create_refresh_interactive_marker()

    def create_refresh_interactive_marker(self):
        int_marker = interactive_marker('behavior_server_refresh', ([0,0,1.], [0,0,0,1]), .2)
        int_marker.header.frame_id = 'torso_lift_link'
        int_marker.description = ''
        int_marker.controls.append(make_sphere_control(int_marker.name, .2))

        menu_handler = mh.MenuHandler()
        menu_handler.insert('Rescan Behaviors', parent=None, callback=self.refresh_actions_tree)
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.name = 'menu_rescan'
        menu_control.markers.append(copy.deepcopy(int_marker.controls[0].markers[0]))
        menu_control.always_visible = True
        int_marker.controls.append(menu_control)

        self.marker_server_lock.acquire()
        self.marker_server.insert(int_marker, None)
        menu_handler.apply(self.marker_server, int_marker.name)
        self.marker_server_lock.release()

    def get_actions_tree(self):
        return self.actions_tree

    def start_marker_server(self):
        self.marker_server = ims.InteractiveMarkerServer(self.SERVER_NAME)

        self.action_marker_manager = ActionMarkersManager(self.action_tag_database_name, self.marker_server_lock, 
                self, self.marker_server, self.actions_db_changed_cb, self.tf_listener)
        self.insert_database_actions()
        self.action_marker_manager.update_behavior_menus()

        self.ar_marker_manager = ARMarkersManager(self.ar_tag_database_name, self.action_marker_manager, self.marker_server_lock, 
                self.marker_server, self.tf_listener, self.broadcaster)

        #Subscribe to AR Pose to determine marker visibility
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", ar_msg.ARMarkers, self.ar_marker_cb)
        rospy.loginfo('Ready!')

    #For listing / serving actions
    def start_list_service(self):
        #self.main_dir_watcher = ras.WatchDirectory(self.path_to_rcommander_files, self.main_directory_watch_cb)
        rospy.Service('list_rcommander_actions', ActionInfo, self.list_action_cb)

    def start_execution_action_server(self):
        self.actserv = actionlib.SimpleActionServer('run_rcommander_action_web', rmsg.RunScriptAction, 
                            execute_cb=self.execute_cb, auto_start=False)
        self.actserv.start()

    def list_action_cb(self, req):
        path = req.path
        if path == '.':
            path = self.path_to_rcommander_files
        fnames, fpaths, anames, apaths = self.list_actions_and_folders_at_path(path, self.actions_tree)
        forder = np.argsort(fnames)
        fnames = np.array(fnames)[forder].tolist()
        fpaths = np.array(fpaths)[forder].tolist()
        
        aorder = np.argsort(anames)
        anames = np.array(anames)[aorder].tolist()
        apaths = np.array(apaths)[aorder].tolist()

        rospy.loginfo('responded to %s' % path)
        return ActionInfoResponse(fnames, fpaths, anames, apaths)

    def list_actions_and_folders_at_path(self, path, actions_tree):
        path = os.path.normpath(path)
        fnames, fpaths = [], []
        anames, apaths = [], []

        splitted = path.split(os.path.sep)
        if actions_tree['path'] == splitted[0]:
            if len(splitted) == 1:
                for a in actions_tree['actions']:
                    if isinstance(a, dict):
                        fnames.append(a['path'])
                        fpaths.append(os.path.join(path, a['path']))
                    else:
                        anames.append(os.path.split(a)[1])
                        apaths.append(a)
                return fnames, fpaths, anames, apaths
            else:
                for a in actions_tree['actions']:
                    if isinstance(a, dict) and a['path'] == splitted[1]:
                        fnames, fpaths, anames, apaths = self.list_actions_and_folders_at_path(os.path.join(*splitted[1:]), a)
                        fpaths = [os.path.join(splitted[0], fn) for fn in fpaths]
                        return fnames, fpaths, anames, apaths 

        return fnames, fpaths, anames, apaths

    def actions_db_changed_cb(self):
        loc_folder, loc_idx = find_folder_idx(self.actions_tree, 'Locations')
        self.actions_tree['actions'].pop(loc_idx)
        self.insert_locations_folder()
        self.insert_database_actions()

    def insert_locations_folder(self):
        loc_folder = find_folder(self.actions_tree, 'Locations')
        if loc_folder == None:
            self.actions_tree['actions'].append({'path':'Locations', 'actions':[]})

    def insert_database_actions(self):
        #loaded actions is keyed with the full behavior path
        locations_tree = find_folder(self.actions_tree, 'Locations')
        #insert individual action from db
        action_db = self.action_marker_manager.marker_db
        for actionid in action_db.ids():
            self._insert_database_action(actionid, locations_tree)

    def _insert_database_action(self, actionid, locations_tree=None):
        if locations_tree == None:
            locations_tree = find_folder(self.actions_tree, 'Locations')

        entry = self.action_marker_manager.marker_db.get(actionid)
        if entry['behavior_path'] != None:
            action_path = self._behavior_path_to_location_path(entry['behavior_path'], entry['tagid'])
            rospy.loginfo('Inserted %s into loaded_actions.' % action_path)
            self.loaded_actions[action_path] = ft.partial(self._execute_database_action_cb, actionid)
            locations_tree['actions'] += [action_path]

    def _behavior_path_to_location_path(self, behavior_path, tagid):
        relative_path = behavior_path.replace(self.path_to_rcommander_files, '')
        root_path = ''
        decomposed_name = relative_path.split(pt.sep)
        behavior_name = decomposed_name[-1]
        for p in decomposed_name[1:-1]:
            root_path = pt.join(root_path, p)
        root_path = pt.join(root_path, tag_name(tagid, behavior_name))
        return pt.join(self.path_to_rcommander_files, pt.join('Locations', root_path))

    def execute_cb(self, goal):
        rospy.loginfo('Executing: ' + goal.action_path)
        if hasattr(self.loaded_actions[goal.action_path], '__call__'):
            self.loaded_actions[goal.action_path]()
        else:
            self.loaded_actions[goal.action_path]['scripted_action_server'].execute(self.actserv)

    def _execute_database_action_cb(self, actionid):
        entry = self.action_marker_manager.marker_db.get(actionid)
        self.action_marker_manager.set_task_frame(actionid)
        self.publish_task_frame_transform()
        self.loaded_actions[entry['behavior_path']]['scripted_action_server'].execute(self.actserv)
        self.action_marker_manager.set_task_frame(None) #This will stop the publishing process

    def ar_marker_cb(self, msg):
        #Filter out markers detected as being behind head
        valid_markers = []
        for marker in msg.markers:
            if marker.pose.pose.position.z > 0:
                valid_markers.append(marker)
        self.visible_markers = valid_markers

    def _load_action_at_path(self, action):
        rospy.loginfo('Loading ' + action)
        action_path = action
        for i in range(4):
            try:
                load_dict = {'scripted_action_server':  ras.ScriptedActionServer(action, 
                                                                action_path, self.robot),
                             #'watcher': ras.WatchDirectory(action_path, 
                             #    self.sub_directory_changed_cb)
                             }
                rospy.loginfo('Successfully loaded ' + action)
                return load_dict
            except IOError, e:
                rospy.loginfo(str(e))
                rospy.loginfo('IOError encountered, retrying.')
                rospy.sleep(3)

    def sub_directory_changed_cb(self, action_path_name):
        rospy.loginfo("SUBDIRECTORY CHANGED CB %s" % action_path_name)

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

    def refresh_actions_tree(self, feedback):
        self.create_actions_tree()
        self.action_marker_manager.update_behavior_menus()

    def create_actions_tree(self):
        rospy.loginfo('create_actions_tree: rescanning ' + self.path_to_rcommander_files)
        actions = ras.find_all_actions(self.path_to_rcommander_files)
        self.loaded_actions, self.actions_tree = self.load_action_from_found_paths(actions)
        self.insert_locations_folder()

        rospy.loginfo('All actions found\n')
        for k in self.loaded_actions.keys():
            rospy.loginfo('ACTION %s' % k)

    def remove_unused_ar_markers(self):
        markers_changed = False
        #Find unused markers
        visible_ids = [m.id for m in self.visible_markers]

        action_db = self.action_marker_manager.marker_db
        referenced_tagids = set([action_db.get(actionid)['tagid'] for actionid in action_db.ids()])

        ar_db = self.ar_marker_manager.marker_db
        #for every tag in db, if it is unused and not visible, delete
        for tid in ar_db.ids():
            if not (tid in visible_ids) and not (tid in referenced_tagids):
                self.ar_marker_manager.remove_marker(tid)
                markers_changed = True

        return markers_changed

    def publish_task_frame_transform(self):
        mdb = self.action_marker_manager.marker_db
        task_frame_info = self.action_marker_manager.get_current_task_frame()
        if task_frame_info == None:
            return
        frame, loc = task_frame_info
        self.broadcaster.sendTransform(loc[0], loc[1], rospy.Time.now(), 'task_frame', frame)

    def step(self, timer_obj):
        markers_changed = self.ar_marker_manager.update(self.visible_markers)
        markers_changed = markers_changed or self.remove_unused_ar_markers()
        self.publish_task_frame_transform()

        if (self.i % (5*5) == 0):
            self.action_marker_manager.marker_db.save(self.action_tag_database_name)
            self.ar_marker_manager.marker_db.save(self.ar_tag_database_name)
        self.i += 1

        if markers_changed:
            self.marker_server_lock.acquire()
            self.marker_server.applyChanges()
            self.marker_server_lock.release()

    def start(self):
        self.i = 0
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














