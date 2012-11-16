import roslib; roslib.load_manifest("rcommander_ar_tour")
import rospy
import actionlib
import std_msgs.msg as stdm

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh
import ar_pose.msg as ar_msg
import geometry_msgs.msg as gmsg

import sys
from rcommander_web.srv import ActionInfo, ActionInfoResponse
import rcommander_ar_tour.srv as rsrv
import geometry_msgs.msg as gmsg

import rcommander_web.msg as rmsg
import rcommander_ar_tour.msg as atmsg
import pypr2.tf_utils as tfu

from PyQt4 import QtCore
import os
import os.path as pt
import numpy as np
import inspect as insp

import detect_robot_move as drm
import rcommander_web.rcommander_auto_server as ras
import tf
import tf.transformations as tr
import cPickle as pk
import functools as ft

import re
import copy
from threading import RLock
import pdb
import glob

DEFAULT_LOC = [[0.,0.,0.], [0.,0.,0.,1.]]
HEAD_MARKER_LOC = ([0,0,1.], 
        tr.quaternion_from_euler(0, 0, np.radians(180.)))
MAX_MARKER_DIST = 1.8

## Create the description that hovers near atag in rviz
# @param tagid number
# @param behavior_name string
# @return string
def tag_name(tagid, behavior_name):
    if tagid != None:
        return '%s (Tag #%d)' % (behavior_name, tagid)
    else:
        return '%s' % (behavior_name)

## Takes an interactive marker feedback and turns it into a string
# @param ftype feedback type (ims.InteractiveMarkerFeedback)
# @return string
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

## Creates an interactive marker
# @param name marker name
# @param pose 2-tuple with translation (3 element list) and rotation
#                   (4 element list)
# @param scale scale of marker
# @return InteractiveMarker object
def interactive_marker(name, pose, scale):
    int_marker = ims.InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.pose.position.x = pose[0][0]
    int_marker.pose.position.y = pose[0][1]
    int_marker.pose.position.z = pose[0][2]
    int_marker.pose.orientation.x = pose[1][0]
    int_marker.pose.orientation.y = pose[1][1]
    int_marker.pose.orientation.z = pose[1][2]
    int_marker.pose.orientation.w = pose[1][3]
    
    int_marker.scale = scale
    int_marker.name = name
    int_marker.description = name
    return int_marker

## Create a Marker object
# @param scale size of marker
# @return Marker object
def make_rviz_marker(scale):
    marker = ims.Marker()
    marker.type = ims.Marker.SPHERE
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color = stdm.ColorRGBA(.5,.5,.5,1)
    return marker

## Make an interactive control marker with a sphere at the center
# @param name name of control
# @param scale size of marker
# @return InteractiveMarkerControl object
def make_sphere_control(name, scale):
    control =  ims.InteractiveMarkerControl()
    control.name = name + '_sphere'
    control.always_visible = True
    control.markers.append(make_rviz_marker(scale))
    control.interaction_mode = ims.InteractiveMarkerControl.BUTTON
    return control

## Make an interactive control marker with a given orientaion
# @param orientation a 4 element list representing a quaternion
# @return InteractiveMarkerControl object
def make_control_marker(orientation=[0,0,0,1.]):
    control = ims.InteractiveMarkerControl()
    control.orientation.x = orientation[0]
    control.orientation.y = orientation[1]
    control.orientation.z = orientation[2]
    control.orientation.w = orientation[3]
    control.interaction_mode = ims.InteractiveMarkerControl.MOVE_AXIS
    return control

## Make controls to move interactive marker along xyz axes
# @return a list of 3 InteractiveMarkerControl objects
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

## Make controls for orientation
# @param name name prefix for InteractiveMarkerControls
# @return a list of 3 interactive marker control objects
def make_orientation_controls(name):
    controls = make_directional_controls(name + '_rotate')
    controls[0].interaction_mode = ims.InteractiveMarkerControl.ROTATE_AXIS
    controls[1].interaction_mode = ims.InteractiveMarkerControl.ROTATE_AXIS
    controls[2].interaction_mode = ims.InteractiveMarkerControl.ROTATE_AXIS
    return controls

## Take a Pose message and turns it into a tf tupple
# @return a 2 tuple with translation and rotation
def pose_to_tup(p):
    return [p.position.x, p.position.y, p.position.z], \
            [p.orientation.x, p.orientation.y, \
            p.orientation.z, p.orientation.w]

## Take a tuple and turns it into a Pose message
# @param p A 2 tuple with translation and rotation
# @return Pose message
def tup_to_pose(p):
    pose = gmsg.Pose()
    pose.position.x = p[0][0]
    pose.position.y = p[0][1]
    pose.position.z = p[0][2]
    pose.orientation.x = p[1][0]
    pose.orientation.y = p[1][1]
    pose.orientation.z = p[1][2]
    pose.orientation.w = p[1][3]
    return pose

## Makes a menu handler from an action tree
# @param actions_tree Structure returned by
#   BehaviorServer.list_actions_and_folders_at_path.
#   Has the format {'path':   full path 'actions': [{another folder},
#                   {another folder2}, rcom_file}
# @param callback A function of the form: f(menu_item, full_action_path).
# @param handler A interactive_markers.menu_handler.MenuHandler object.
# @return handler A interactive_markers.menu_handler.MenuHandler object.
def menu_handler_from_action_tree(actions_tree, callback, handler=None):
    if handler == None:
        handler = mh.MenuHandler()
    menu_handler_from_action_tree_helper(handler, actions_tree, callback)
    return handler

## Helper method for menu_handler_from_action_tree
def menu_handler_from_action_tree_helper(handler, actions_tree, callback, 
        parent=None):
    base_path = actions_tree['path']
    for action in actions_tree['actions']:
        if isinstance(action, dict):
            submenu = handler.insert(action['path'], parent=parent)
            menu_handler_from_action_tree_helper(handler, action, callback,\
                    parent=submenu)
        else:
            action_name = os.path.split(action)[1]
            handler.insert(action_name, parent=parent, \
                    callback=ft.partial(callback, action_name, action))

## Load pickle databases from disk
# @param name Name/path of file.
# @param db_class A class that inherits from the class Database
# @return An object of the type db_class.
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

## Look up transform without all the exceptions.
# @param listener TFListener object.
# @param aframe a frame name.
# @param bframe another frame name.
# @erturn None or a TF tuple.
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

## Take a function of the form f() and wraps it with TF exception handlers
# @param func function of the form f()
# @return whatever f returns or None if exception encountered
def catch_silly_tf_exceptions_and_ignore_them(func):
    try:
        result = func()
        return result

    except tf.ExtrapolationException, e:
        rospy.logdebug( 'exception %s' % str(e))
        return None

    except tf.ConnectivityException, e:
        rospy.logdebug( 'exception %s' % str(e))
        return None

    except tf.LookupException, e:
        rospy.logdebug( 'exception %s' % str(e.__class__))
        return None

## Helper function to list non-empty folders in an actiontree
# @param actions_tree Structure returned by
#   BehaviorServer.list_actions_and_folders_at_path.
#   Has the format {'path':   full path 'actions': [{another folder},
#                   {another folder2}, rcom_file}
# @param path A string representing a path in the actions tree.
def find_nested_folder(actions_tree, path):
    if len(path) <= 0:
        return actions_tree
    folder = find_folder(actions_tree, path[0])
    if folder != None:
        return find_nested_folder(folder, path[1:])

## Given a folder name, find the action instance and index that
# represents that folder
# @param actions_tree Structure returned by
#   BehaviorServer.list_actions_and_folders_at_path.
#   Has the format {'path':   full path 'actions': [{another folder},
#                   {another folder2}, rcom_file}
# @param folder_name Folder name (string)
# @return a tuple consisting of an action dictionary, and index (integer)
def find_folder(actions_tree, folder_name):
    return find_folder_idx(actions_tree, folder_name)[0]

## Helper for the above function
# @return a tuple consisting of an action dictionary, and index (integer)
def find_folder_idx(actions_tree, folder_name):
    for idx, action in enumerate(actions_tree['actions']):
        if isinstance(action, dict) and action['path'] == folder_name:
            return action, idx
    return None, None

## Represents a generic RCommander database. Store entries keyed on an
# actionid, allow definition, retrieval of properties on those
# actionid's.  Also provides mechanisms for saving, loading and id
# based operations. Needs to be subclassed to be useful.
class Database:

    ## Constructor
    def __init__(self):
       self.modified = True
       self.database = {}

    def get(self, an_id):
        return copy.deepcopy(self.database[an_id])

    def has_id(self, an_id):
        return an_id in self.database.keys()

    def ids(self):
        return self.database.keys()

    ## Save database to disk
    # @param name filename to use for saving to disk
    def save(self, name):
        if self.modified:
            self.modified = False
            pickle_file = open(name, 'w')
            pk.dump(self.database, pickle_file)
            pickle_file.close()

    def remove(self, an_id):
        self.database.pop(an_id)
        self.modified = True

    ## Sets the property
    # @param actionid id to save property to
    # @param prop_name name of property
    # @param value value of property
    def set_property(self, actionid, prop_name, value):
        self.database[actionid][prop_name] = value

    ## Checks for the existence of property
    # @param actionid id to check for property
    # @param prop_name name of property to check
    def has_property(self, actionid, prop_name):
        return self.database[actionid].has_key(prop_name)

## Database to save actions (or task frames associated with a loaded
# ROS Commander behaviors)
class ActionDatabase(Database):
       
   ## Constructor
   def __init__(self):
        Database.__init__(self)

   ## Insert a new tag + frame pair into database
   def insert(self, name, frame, tagid, 
           action_pose=DEFAULT_LOC, behavior_path=None):
       actionid = name + str(rospy.get_rostime().to_time())
       self.database[actionid] = {'name': name, 'frame': frame, 
                                  'loc': action_pose, 
                                  'behavior_path': behavior_path,
                                  'tagid': tagid}
       self.modified = True
       return actionid

   ## Update the id of the frame the 'loc' property is defined in.
   # @param actionid ID of action.
   # @param frame Frame name.
   def update_frame(self, actionid, frame):
       self.database[actionid]['frame'] = frame
       self.modified = True

   ## Update the behavior associated with given action id
   # @param actionid ID of action.
   # @param behavior Behavior path.
   def update_behavior(self, actionid, behavior):
       self.database[actionid]['behavior_path'] = behavior
       self.modified = True

   ## Update the location of the given actionid
   # @param actionid ID of action
   # @param loc Pose as a tf tuple (trans, rotation quaternion)
   def update_loc(self, actionid, loc):
       self.database[actionid]['loc'] = loc
       self.modified = True


## Represents an action (or behavior + task frame) as an interactive marker
class ActionMarker:

    ## Constructor
    # the marker represents itself in the /map frame internally but presents
    # an interface in its defined frame
    # @param manager Instance of ActionMarkersManager
    # @param actionid ID of this action.
    # @param location_in_frame Location as a TF tuple.
    # @param frame frame name that location_in_frame is defined in
    # @param tagid ID of the AR Tag that the frame is defined wrt.
    # @apram behavior_name Path of behavior tested.
    # @param marker_server Instance of interactive marker server.
    # @param server_lock RLock to lock the ActionServer
    # @param tf_listener TF Listener
    def __init__(self, manager, actionid, location_in_frame, frame, 
            tagid, behavior_name, marker_server, server_lock, tf_listener, 
            menu_handler=None):
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
        self.movement_controls = True
        self._make_marker()

    ## Add "delete" option to menu handler.
    # @param menu_handler a MenuHandler instance
    def _add_delete_option(self, menu_handler):
        menu_handler.insert('-----------------', parent=None, callback=None)
        menu_handler.insert('Toggle Controls', parent=None, 
                callback=self.toggle_controls_cb)
        menu_handler.insert('Delete', parent=None, 
                callback=self.delete_action_cb)

    ## Callback for toggle controls option
    # @param feedback Feedback parameter from InteractiveMarker
    def toggle_controls_cb(self, feedback):
        self.remove()
        self.movement_controls = not self.movement_controls
        self._make_marker()
        self.server_lock.acquire()
        self.marker_server.applyChanges()
        self.server_lock.release()

    ## Callback for train option (not in list due to user study)
    def train_cb(self, feedback):
        rospy.loginfo('train_cb called %s' % str(feedback))
        self.manager.train_publisher.publish(atmsg.TrainAction(self.actionid))

    ## Callback for delete action
    def delete_action_cb(self, feedback):
        self.manager.delete_marker_cb(self.actionid)

    ## Selects this action as THE primary task frame
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

    ## Remove this marker from interactive marker display (does not
    # delete from db)
    def remove(self):
        if self.marker_obj != None:
            self.server_lock.acquire()
            self.marker_server.erase(self.marker_name)
            self.server_lock.release()

            self.marker_obj = None
            self.marker_name = None

    ## Update databse locations (two tf tuple)
    # @param update_database call whether to update the marker database
    # with this new pose
    def update(self, pose_in_defined_frame, update_database):
        self.manager.update_loc(self.actionid, 
                pose_to_tup(pose_in_defined_frame), update_database)
        self.marker_obj.pose = pose_in_defined_frame
        self.location_in_frame = pose_to_tup(pose_in_defined_frame)

    ## Convert a point in the frame defined by this marker into the
    # 'map' frame.
    def _to_map_frame(self, p_ar):
        m_ar = tfu.tf_as_matrix(p_ar)
        self.tf_listener.waitForTransform('map', self.frame, 
                rospy.Time.now(), rospy.Duration(10))
        map_T_ar = tfu.transform('map', self.frame, self.tf_listener, t=0)
        p_map = tfu.matrix_as_tf(map_T_ar * m_ar)
        return p_map

    ## Creates the interactive marker that represents this class
    def _make_marker(self): 
        if self.is_current_task_frame:
            scale = .3
            color = stdm.ColorRGBA(1,0,0,.4)
        else:
            scale = .2
            color = stdm.ColorRGBA(.5,.5,.5,.4)

        self.marker_name = 'action_' + self.actionid
        pose = self.location_in_frame

        int_marker = interactive_marker(self.marker_name, 
                (pose[0], pose[1]), scale)
        int_marker.header.frame_id = self.frame 
        int_marker.description = self._make_description()
        int_marker.controls.append(make_sphere_control(\
                self.marker_name + '_1', scale/8.))
        int_marker.controls.append(make_sphere_control(\
                self.marker_name + '_2', scale))
        int_marker.controls[1].markers[0].color = color
        if self.movement_controls:
            int_marker.controls += make_directional_controls(self.marker_name)
            int_marker.controls += make_orientation_controls(self.marker_name)

        self.server_lock.acquire()
        self.marker_server.insert(int_marker, self.marker_cb)
        self.server_lock.release()
        self.marker_obj = int_marker
        if self.menu_handler != None:
            self._make_menu()
        else:
            self._make_empty_menu()

    ## Makes an empty menu (without any actions)
    def _make_empty_menu(self):
        self.menu_handler = mh.MenuHandler()
        self._add_delete_option(self.menu_handler)
        self._make_menu()

    ## Makes a menu with a list of AR tag actions
    def _make_menu(self):
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = 'menu_' + self.actionid
        menu_control.markers.append(copy.deepcopy(\
                self.marker_obj.controls[0].markers[0]))
        menu_control.always_visible = True
        self.marker_obj.controls.append(copy.deepcopy(menu_control))

        self.server_lock.acquire()
        self.marker_server.insert(self.marker_obj, self.marker_cb)
        self.menu_handler.apply(self.marker_server, self.marker_obj.name)
        self.server_lock.release()

    ## Use an existing menu_handler as this object's menu handler
    # @param MenuHandler object
    def set_menu(self, menu_handler):
        self._add_delete_option(menu_handler)
        self.menu_handler = menu_handler

        #erase current marker
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        self.server_lock.release()

        #insert it with a menu_handler
        self._make_menu()

    ## Creates text descriptions for this marker that float above it in rviz
    def _make_description(self):
        if self.behavior_name != None:
            return tag_name(self.tagid, self.behavior_name)
        else:
            return ''

    ## Update the name of this marker (used after user binds it to a
    # behavior
    def update_name(self, behavior_name):
        self.behavior_name = behavior_name
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        
        self.marker_obj.description = self._make_description()
        self.marker_server.insert(self.marker_obj, self.marker_cb)
        self.marker_server.applyChanges()
        self.server_lock.release()

    ## Callback from InteractiveMarkerServer when user clicks on this marker
    def marker_cb(self, feedback):
        if feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            self.update(feedback.pose, True)

        sphere_match = re.search('_sphere$', feedback.control_name)
        if (sphere_match != None) and feedback.event_type \
                == ims.InteractiveMarkerFeedback.MOUSE_DOWN:
            self.manager.set_task_frame(self.actionid)
            self.server_lock.acquire()
            self.marker_server.applyChanges()
            self.server_lock.release()

## Keeps track of ActionMarkers
class ActionMarkersManager:

    ## Constructor
    def __init__(self, action_marker_database_name, server_lock, 
           behavior_server, marker_server, actions_db_changed_cb, tf_listener):
        self.server_lock = server_lock
        self.behavior_server = behavior_server
        self.marker_server = marker_server
        self.actions_db_changed_cb = actions_db_changed_cb
        self.tf_listener = tf_listener
        self.train_publisher = rospy.Publisher('train_action', 
                atmsg.TrainAction)

        #Database and marker list has to be in sync!
        self.marker_db = Database_load(action_marker_database_name, 
                ActionDatabase)
        self.markers = {}

        self.server_lock.acquire()
        for action_id in self.marker_db.ids():
            self._create_marker(action_id)
        self.marker_server.applyChanges()
        self.server_lock.release()

    ## Callback for when user clicks on broadcast point on interactive marker
    # point cloud. Not hooked up right now for user study...
    # @param posestamped PoseStamped message when user clicks on the point cloud.
    def cloud_clicked_cb(self, posestamped):
        marker_obj = None
        for actionid in self.markers.keys():
            marker = self.markers[actionid]
            if marker.is_current_task_frame:
                marker_obj = marker

        def transformPose(frame):
            self.tf_listener.waitForTransform(frame, 
                    posestamped.header.frame_id, rospy.Time.now(), 
                    rospy.Duration(10))
            return self.tf_listener.transformPose(frame, posestamped)

        if marker_obj != None:
            fps = catch_silly_tf_exceptions_and_ignore_them(
                    ft.partial(transformPose, marker_obj.frame))

            if fps != None:
                current_orientation = self.marker_db.get(\
                        marker_obj.actionid)['loc'][1]
                fps.pose.orientation.x = current_orientation[0]
                fps.pose.orientation.y = current_orientation[1]
                fps.pose.orientation.z = current_orientation[2]
                fps.pose.orientation.w = current_orientation[3]
                marker_obj.update(fps.pose, True)

    ## Get the ActionMarker current selected as the task frame (highlighted red)
    # @return actionid ID of ActionMarker, frame marker is defined in, location
    #           of marker
    #           or None
    def get_current_task_frame(self):
        for actionid in self.markers.keys():
            marker = self.markers[actionid]
            if marker.is_current_task_frame:
                return actionid, marker.frame, marker.location_in_frame
        return None

    ## Get just the name part of a behavior from a behavior's file path
    # @param rec an ActionDatabase record
    # @return a string (or None)
    def _get_behavior_name(self, rec):
        if rec['behavior_path'] != None:
            return pt.split(rec['behavior_path'])[1]
        else:
            return None

    ## Creates an action (defined as a frame associated with a behavior)
    # @param parent_frame Frame that this action's frame is defined with respect to (string).
    # @param tagid ID of the AR Tag that this action is defined on (integer)
    # @param loc location of this action's frame (as a tf tuple)
    # @param name Prefix name of action (string)
    def create_action(self, parent_frame, tagid, loc=DEFAULT_LOC, 
            name='action'):
        actionid = self.marker_db.insert(name, parent_frame, tagid, 
                action_pose=loc)
        rec = self.marker_db.get(actionid)
        self._create_marker(actionid)
        self.server_lock.acquire()
        self.marker_server.applyChanges()
        self.server_lock.release()
        return actionid
    
    ## Creates an ActionMarker when given an actionid using the ActionDatabase
    # @param actionid Action ID of marker to create (has to be in database) (a string)
    def _create_marker(self, actionid):
        rec = self.marker_db.get(actionid)
        self.markers[actionid] = ActionMarker(self, actionid, rec['loc'], 
                rec['frame'], rec['tagid'], self._get_behavior_name(rec), 
                self.marker_server, self.server_lock, self.tf_listener, 
                self._create_menu_handler(actionid))

    ## Creates a menu handler given an actionid
    # @param actionid Action ID of marker to create menu handler on
    # @return a MenuHandler
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

    ## Deletes marker with given action ID
    # @param actionid Action ID to delete.
    def delete_marker_cb(self, actionid):
        if self.marker_db.has_property(actionid, 'complement'):
            cactionid = self.marker_db.get(actionid)['complement']
            self.markers[cactionid].remove()
            self.markers.pop(cactionid)
            self.marker_db.remove(cactionid)

        self.markers[actionid].remove()
        self.markers.pop(actionid)
        self.marker_db.remove(actionid)

        self.server_lock.acquire()
        self.marker_server.applyChanges()
        self.server_lock.release()
        self.actions_db_changed_cb()

    ## Callback from ARTagMarker to perform selection
    # @param actionid Action ID to delete
    def set_task_frame(self, actionid):
        for k in self.markers.keys():
            self.markers[k].set_selected(k == actionid)

    ## Callback from ARTagMarker to update location of action
    # @param actionid Action ID to delete
    # @param loc TF tuple of translation and rotation.
    # @param update_database whether to allow updating of database or not (boolean) 
    #           TODO: remove this
    def update_loc(self, actionid, loc, update_database=True):
        if update_database:
            self.marker_db.update_loc(actionid, loc)

    ## Update menus containing lists of behaviors
    def update_behavior_menus(self):
        self.server_lock.acquire()
        for actionid in self.markers.keys():
            menu_handler = self._create_menu_handler(actionid)
            self.markers[actionid].set_menu(menu_handler)
        self.marker_server.applyChanges()
        self.server_lock.release()

    ## Callback for when an action is selected on menu created by update_behavior_menus
    # @param actionid Action ID of marker
    # @param menu_item menu item clicked on
    # @param full_action_path Full path of action clicked on.
    # @param int_feedback Interactive marker's feedback object.
    def _action_selection_menu_cb(self, actionid, menu_item, 
            full_action_path, int_feedback):
        rospy.loginfo('Clicked behavior %s on action %s' \
                % (full_action_path, actionid))
        self.marker_db.update_behavior(actionid, full_action_path)
        self.markers[actionid].update_name(pt.split(full_action_path)[1])
        self.actions_db_changed_cb()

    ## Gets the ActionMarker associated with an actionid
    # @param actionid
    def get_marker(self, actionid):
        return self.markers[actionid]

## A database of where AR tags are in the map frame
class ARTagDatabase(Database):

    ## Constructor
    def __init__(self):
        Database.__init__(self)

    ## Store the location of an AR Tag
    # @param tag_id ID of tag (integer)
    # @param tag_location TF tuple of translation rotation
    def set_location(self, tag_id, tag_location): 
        self.database[tag_id] = tag_location 
        self.modified = True
        return tag_id

##
# User can create multiple different action markers from ar tag markers
class ARTagMarker:

    ## Constructor
    # @param tagid ID of AR Tag (integer).
    # @param pose_in_map_frame Tuple in TF format (translation, rotation).
    # @param action_marker_manager an ActionMarkersManager instance (used for
    #                   creating new actions)
    def __init__(self, tagid, pose_in_map_frame, action_marker_manager, 
            server_lock, marker_server, scale=.2):
        self.tagid = tagid
        self.action_marker_manager = action_marker_manager
        self.server_lock = server_lock
        self.marker_server = marker_server
        self.marker_name = 'ar_marker_' + str(self.tagid)

        int_marker = interactive_marker(self.marker_name, pose_in_map_frame, 
                scale)
        int_marker.scale = .6
        self.marker_obj = int_marker
        int_marker.description = 'Tag #%d' % self.tagid

        #int_marker.controls += []
        #int_marker.controls[0].markers[0].color = stdm.ColorRGBA(0,1,0,.5)
        sph = make_sphere_control(self.marker_name, scale)
        sph.markers[0].color = stdm.ColorRGBA(0,1,0,.5)

        self.menu = mh.MenuHandler()
        self.menu.insert('Create New Action', parent=None, 
                callback=self.create_new_action_cb)
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

    ## Callback for create new actions button
    # @param feedback Feedback object from interactive marker server
    def create_new_action_cb(self, feedback):
        #print 'create_new_action_cb', feedback
        rospy.loginfo('New action created! Tag id %d.' % self.tagid)
        self.action_marker_manager.create_action(ar_frame_name(self.tagid), 
                self.tagid)

    ## Ignore feedback for this marker
    def marker_cb(self, feedback):
        pass

    ## Update the pose of this AR Tag wrt to the map frame
    def update(self, pose_in_map_frame):
        pose = tup_to_pose(pose_in_map_frame)

        self.server_lock.acquire()
        self.marker_server.setPose(self.marker_name, pose)
        self.server_lock.release()

    ## Remove this marker from view
    def remove(self):
        self.server_lock.acquire()
        self.marker_server.erase(self.marker_name)
        self.server_lock.release()

def ar_frame_name(tagid):
    return '4x4_' + str(tagid)

## Allows managing of interactive markers based on detection by ARToolKit Tag detectors 
class ARMarkersManager:

    ## Constructor
    # @param ar_tag_database_name Filename of the AR tag database.
    # @param action_marker_manager ActionMarkersManager instance
    # @param server_lock RLock instance to lock marker_server
    # @param marker_server InteractiveMarkerServer instance
    # @param tf_listener TFListener instance
    # @param tf_broadcaster TransformBroadcaster instance
    def __init__(self, ar_tag_database_name, action_marker_manager, 
            server_lock, marker_server, tf_listener, tf_broadcaster):

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

    ## Create an AR Tag marker (green sphere)
    def create_marker(self, tagid, location):
        self.markers[tagid] = ARTagMarker(tagid, location, 
                self.action_marker_manager, self.server_lock, 
                self.marker_server)

    ## Remove an AR Tag marker from viz
    def remove_marker(self, tagid):
        self.markers[tagid].remove()
        self.markers.pop(tagid)
        self.marker_db.remove(tagid)

    ## Update the list of visible AR Tag markers with list of currently detected markers.
    # @param visible_markers a list of ARMarkers messages.
    def update(self, visible_markers):
        visible_ids = [m.id for m in visible_markers]
        markers_changed = False

        if not self.robot_movement_detector.is_moving():
            for mid in visible_ids:
                fn = ar_frame_name(mid)
                ar_location = lookup_transform(self.tf_listener, 'map', fn)
                if ar_location != None:
                    if not self.markers.has_key(mid):
                        self.create_marker(mid, ar_location)
                        self.markers[mid].update(ar_location)
                        markers_changed = True
                    else:
                        self.markers[mid].update(ar_location)
                        markers_changed = True
                    self.marker_db.set_location(mid, ar_location)

        for mid in self.markers.keys():
            if mid in visible_ids:
                continue
            position, orientation = self.marker_db.get(mid)
            self.broadcaster.sendTransform(position, orientation, 
                    rospy.Time.now(), ar_frame_name(mid), '/map')

        return markers_changed

## Main class for serving up, visualizing and executing behaviors.
class BehaviorServer:

    ## Constructor
    # @param action_tag_database_name Filename of an ActionDatabase.
    # @param ar_tag_database_name Filename of ARTagDatabase. 
    # @param path_to_rcommander_files Path to saved behaviors.
    # @param tf_listener TFListener object.
    # @param robot Robot object (like in RCommander)
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

        self.actions_tree = {'path': self.path_to_rcommander_files, 
                             'actions':[]}
        self.actserv_runweb      = None
        self.actserv_runactionid = None
        self.head_menu_marker = None

        self.create_actions_tree()
        self.start_marker_server()
        self.start_services()
        self.start_execution_action_server()
        self.create_head_menu()

    ## Callback for selection of a learnable behavior 
    # @param menu_item name of action clicked on
    # @param full_action_path full file path to action
    # @param int_feedback Interactive Marker feedback.
    def learnable_behavior_menu_cb(self, menu_item, full_action_path, 
            int_feedback):
        clicked_action = full_action_path
        folder, action_name = pt.split(full_action_path)
        all_actions = glob.glob(pt.join(folder, '*'))
        nonmatchings = []

        for a in all_actions:
            if a != clicked_action:
                nonmatchings.append(a)

        if len(nonmatchings) > 1:
            rospy.logerr('Found too many items in ' +
                    '%s.  There can only be two behaviors.' % folder)
            return

        clicked_complement = nonmatchings[0]

        rospy.loginfo('clicked %s, complement is %s' \
                % (clicked_action, clicked_complement))

        p_tll = tfu.tf_as_matrix(HEAD_MARKER_LOC)
        self.tf_listener.waitForTransform('map', 'torso_lift_link', 
                rospy.Time.now(), rospy.Duration(10))
        map_T_tll = tfu.transform('map', 'torso_lift_link', 
                self.tf_listener, t=0)
        p_map = tfu.matrix_as_tf(map_T_tll * p_tll)

        #Create two markers
        
        actionid = self.action_marker_manager.create_action('map', 
                tagid=None, loc=p_map, name=pt.split(clicked_action)[1])
        self.action_marker_manager._action_selection_menu_cb(actionid, 
                            menu_item, clicked_action, int_feedback)

        cactionid = self.action_marker_manager.create_action('map', 
                tagid=None, loc=p_map, name=pt.split(clicked_complement)[1])
        self.action_marker_manager._action_selection_menu_cb(cactionid, 
                            menu_item, clicked_complement, int_feedback)

        self.action_marker_manager.marker_db.set_property(actionid, 
                'complement', cactionid)
        self.action_marker_manager.marker_db.set_property(cactionid, 
                'complement', actionid)


    ## Creates the menu that floats on top of the robot's head (as a white sphere)
    def create_head_menu(self):
        if self.head_menu_marker != None:
            self.marker_server_lock.acquire()
            self.marker_server.erase('behavior_server_refresh')
            self.marker_server_lock.release()

        int_marker = interactive_marker('behavior_server_refresh', 
                HEAD_MARKER_LOC, .2)
        int_marker.header.frame_id = 'torso_lift_link'
        int_marker.description = ''
        int_marker.controls.append(make_sphere_control(int_marker.name, .2))

        menu_handler = mh.MenuHandler()
        menu_handler.insert('Rescan Behaviors', parent=None, 
                callback=self.rescan_behaviors_cb)
        menu_handler.insert('-----------------', parent=None, 
                callback=None)

        #Insert calls to learnable behaviors
        behavior_folder = find_folder(self.actions_tree, 'Behaviors')
        if behavior_folder != None:
            learnable_folder = copy.deepcopy(find_folder(behavior_folder, 
                'Learnable'))
            if learnable_folder != None:
                tree = {'path':'root', 'actions':[learnable_folder]}
                menu_handler_from_action_tree(tree, 
                        self.learnable_behavior_menu_cb, menu_handler)

        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.name = 'menu_rescan'
        menu_control.markers.append(copy.deepcopy(\
                int_marker.controls[0].markers[0]))
        menu_control.always_visible = True
        int_marker.controls.append(menu_control)

        self.marker_server_lock.acquire()
        self.marker_server.insert(int_marker, None)
        menu_handler.apply(self.marker_server, int_marker.name)
        self.marker_server.applyChanges()
        self.marker_server_lock.release()
        self.head_menu_marker = int_marker

    ## Gets the actions_tree
    # @return Structure returned by list_actions_and_folders_at_path.
    #   Has the format {'path':   full path 'actions': [{another folder},
    #                   {another folder2}, rcom_file}
    def get_actions_tree(self):
        return self.actions_tree

    ## Start the InteractiveMarkerServer, loads database files, etc.
    def start_marker_server(self):
        self.marker_server = ims.InteractiveMarkerServer(self.SERVER_NAME)

        self.action_marker_manager = ActionMarkersManager(
                self.action_tag_database_name, self.marker_server_lock, 
                self, self.marker_server, self.actions_db_changed_cb, 
                self.tf_listener)
        self.insert_database_actions()
        self.action_marker_manager.update_behavior_menus()

        self.ar_marker_manager = ARMarkersManager(self.ar_tag_database_name, 
                self.action_marker_manager, self.marker_server_lock, 
                self.marker_server, self.tf_listener, self.broadcaster)

        #Subscribe to AR Pose to determine marker visibility
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", 
                ar_msg.ARMarkers, self.ar_marker_cb)
        rospy.loginfo('Ready!')

    ## Start services that this node offers
    # For listing / serving actions
    def start_services(self):
        ## List ROS Commander actions available at given path
        rospy.Service('list_rcommander_actions', ActionInfo, 
                self.list_action_cb)
        ## Get named property given an action ID and attribute name
        rospy.Service('get_behavior_property', rsrv.ActionProperty, 
                self.get_behavior_property_cb)
        ## Set the translation/rotation of given behavior  
        rospy.Service('set_behavior_pose', rsrv.SetBehaviorPose, 
                self.set_behavior_pose_cb)
        ## Get the translation/rotation of given behavior  
        rospy.Service('get_behavior_pose', rsrv.GetBehaviorPose, 
                self.get_behavior_pose_cb)
        ## Get the action ID of the frame currenty designated as the task frame
        rospy.Service('get_active_action_id', rsrv.GetActiveActionID, 
                self.get_active_action_id_cb)

    ## Start the server that executes actions called through the web.
    def start_execution_action_server(self):
        self.actserv_runweb = actionlib.SimpleActionServer(\
                'run_rcommander_action_web', rmsg.RunScriptAction, 
                execute_cb=self.run_action_web_cb, auto_start=False)
        self.actserv_runweb.start()

    ## Callback: Get the action ID of the frame currenty designated as the task frame
    def get_active_action_id_cb(self, req):
        task_frame_info = self.action_marker_manager.get_current_task_frame()
        if task_frame_info != None:
            actionid, _, _ = task_frame_info
            return rsrv.GetActiveActionIDResponse(actionid)
        else:
            return rsrv.GetActiveActionIDResponse('')


    ## Callback: Set the translation/rotation of given behavior  
    def set_behavior_pose_cb(self, req):
        pose = req.posestamped.pose
        actionid = req.actionid
        update_database = req.update_database

        marker_obj = self.action_marker_manager.get_marker(actionid)
        marker_obj.update(pose, update_database)

        self.marker_server_lock.acquire()
        self.marker_server.setPose(marker_obj.marker_name, pose)
        self.marker_server.applyChanges()
        self.marker_server_lock.release()

        return rsrv.SetBehaviorPoseResponse()

    ## Callback: Get the translation/rotation of given behavior  
    def get_behavior_pose_cb(self, req):
        ps = gmsg.PoseStamped()
        ps.pose = tup_to_pose(self.action_marker_manager.marker_db.get(\
                req.actionid)['loc'])
        ps.header.frame_id = self.action_marker_manager.marker_db.get(\
                req.actionid)['frame']
        return rsrv.GetBehaviorPoseResponse(ps)

    ## Callback: Get named property given an action ID and attribute name
    def get_behavior_property_cb(self, req):
        actionid = req.actionid
        prop = req.attribute
        value = self.action_marker_manager.marker_db.get(actionid)[prop]
        if isinstance(value, str):
            return rsrv.ActionPropertyResponse(value)
        else:
            return rsrv.ActionPropertyResponse(pk.dumps(value))

    ## Callback: List ROS Commander actions available at given path
    def list_action_cb(self, req):
        path = req.path
        if path == '.':
            path = self.path_to_rcommander_files
        fnames, fpaths, anames, apaths = \
                self.list_actions_and_folders_at_path(path, self.actions_tree)
        forder = np.argsort(fnames)
        fnames = np.array(fnames)[forder].tolist()
        fpaths = np.array(fpaths)[forder].tolist()
        
        aorder = np.argsort(anames)
        anames = np.array(anames)[aorder].tolist()
        apaths = np.array(apaths)[aorder].tolist()

        rospy.loginfo('responded to %s' % path)
        return ActionInfoResponse(fnames, fpaths, anames, apaths)

    ## Finds valid ROS Comander behaviors given a path
    # @param path file path to actions
    # @param actions_tree tree that 
    #   with elements has the format {'path':   full path 'actions': [{another folder},
    #                   {another folder2}, rcom_file}
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
                        fnames, fpaths, anames, apaths = \
                                self.list_actions_and_folders_at_path(
                                        os.path.join(*splitted[1:]), a)
                        fpaths = [os.path.join(splitted[0], fn) \
                                for fn in fpaths]
                        return fnames, fpaths, anames, apaths 

        return fnames, fpaths, anames, apaths

    
    ## Callback: called by ActionMarkersManager whenever an action disappears from selectable list
    def actions_db_changed_cb(self):
        loc_folder, loc_idx = find_folder_idx(self.actions_tree, 'Locations')
        self.actions_tree['actions'].pop(loc_idx)
        self.insert_locations_folder()
        self.insert_database_actions()

    ## Add the "Locations" folder as a menue
    def insert_locations_folder(self):
        loc_folder = find_folder(self.actions_tree, 'Locations')
        if loc_folder == None:
            self.actions_tree['actions'].append({'path':'Locations', 
                'actions':[]})

    ## Insert actions loaded from on disk databases
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
            action_path = self._behavior_path_to_location_path(
                    entry['behavior_path'], entry['tagid'])
            rospy.loginfo('Inserted %s into loaded_actions.' % action_path)
            self.loaded_actions[action_path] = \
                    {'function': ft.partial(\
                        self._execute_database_action_cb, actionid),
                     'actionid': actionid}
            locations_tree['actions'] += [action_path]

    def _behavior_path_to_location_path(self, behavior_path, tagid):
        relative_path = behavior_path.replace(
                self.path_to_rcommander_files, '')
        root_path = ''
        decomposed_name = relative_path.split(pt.sep)
        behavior_name = decomposed_name[-1]
        for p in decomposed_name[1:-1]:
            root_path = pt.join(root_path, p)
        root_path = pt.join(root_path, tag_name(tagid, behavior_name))
        return pt.join(self.path_to_rcommander_files, 
                pt.join('Locations', root_path))


    def run_action_web_cb(self, req):
        rospy.loginfo('Executing: ' + req.action_path)
        if hasattr(self.loaded_actions[req.action_path]['function'], 
                '__call__'):
            start_time = rospy.get_time()
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('Recording time.')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.loaded_actions[req.action_path]['function'](\
                    self.actserv_runweb)
            end_time = rospy.get_time()
            rospy.loginfo('Took %.3f seconds to run' % (end_time - start_time))


    def _execute_database_action_cb(self, actionid, actserv):
        entry = self.action_marker_manager.marker_db.get(actionid)
        self.action_marker_manager.set_task_frame(actionid)
        self.loaded_actions[entry['behavior_path']]['function'](actserv)
        #This will stop the publishing process
        self.action_marker_manager.set_task_frame(None) 

    def ar_marker_cb(self, msg):
        #Filter out markers detected as being behind head
        valid_markers = []
        for marker in msg.markers:
            if marker.pose.pose.position.z > 0 \
                    and marker.pose.pose.position.z < MAX_MARKER_DIST:
                valid_markers.append(marker)
        self.visible_markers = valid_markers

    def _load_action_at_path(self, action):
        rospy.loginfo('Loading ' + action)
        action_path = action
        for i in range(4):
            try:
                action_server = ras.ScriptedActionServer(action, action_path, 
                        self.robot)
                load_dict = {'scripted_action_server':  action_server,
                             'function': action_server.execute}
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
                sub_loaded_actions, sub_pruned_tree = \
                        self.load_action_from_found_paths(action)
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

    def rescan_behaviors_cb(self, feedback):
        rospy.loginfo('Rescan behaviors called')
        self.create_actions_tree()
        self.create_head_menu()
        self.action_marker_manager.update_behavior_menus()

    def create_actions_tree(self):
        rospy.loginfo('create_actions_tree: rescanning ' \
                + self.path_to_rcommander_files)
        actions = ras.find_all_actions(self.path_to_rcommander_files)
        self.loaded_actions, self.actions_tree = \
                self.load_action_from_found_paths(actions)
        self.insert_locations_folder()

        rospy.loginfo('All actions found\n')
        for k in self.loaded_actions.keys():
            rospy.loginfo('ACTION %s' % k)

    def remove_unused_ar_markers(self):
        markers_changed = False
        #Find unused markers
        visible_ids = [m.id for m in self.visible_markers]

        action_db = self.action_marker_manager.marker_db
        referenced_tagids = set([action_db.get(actionid)['tagid'] \
                for actionid in action_db.ids()])

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
        actionid, frame, loc = task_frame_info
        self.broadcaster.sendTransform(loc[0], loc[1], rospy.Time.now(), 
                'task_frame', frame)

    def step(self, timer_obj):
        markers_changed = self.ar_marker_manager.update(self.visible_markers)
        markers_changed = markers_changed or self.remove_unused_ar_markers()
        self.publish_task_frame_transform()

        if (self.i % (5*5) == 0):
            self.action_marker_manager.marker_db.save(\
                    self.action_tag_database_name)
            self.ar_marker_manager.marker_db.save(self.ar_tag_database_name)
        self.i += 1

        if markers_changed:
            self.marker_server_lock.acquire()
            self.marker_server.applyChanges()
            self.marker_server_lock.release()

    def start(self):
        self.i = 0
        rospy.Timer(rospy.Duration(.1), self.step)


##
# @param server_func, mechanism allowing inherited  servers to run too
def run(robot, tf_listener, action_database_name, 
        ar_tag_database_name, path_to_rcommander_files, server_func):
    import sys
    from PyQt4 import QtCore#, QtGui
    import signal

    app = QtCore.QCoreApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    if server_func != None:
        behavior_server = server_func(action_database_name, 
                ar_tag_database_name, path_to_rcommander_files, 
                tf_listener, robot)
    else:
        behavior_server = BehaviorServer(action_database_name, 
                ar_tag_database_name, path_to_rcommander_files, 
                tf_listener, robot)

    behavior_server.start()
    rospy.loginfo('RCommander AR Tour: Action database name %s' \
            % action_database_name)
    rospy.loginfo('RCommander AR Tour: Tag database name %s' \
            % ar_tag_database_name)
    rospy.loginfo('RCommander AR Tour Server up!')
    app.exec_()














