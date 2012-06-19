import roslib; roslib.load_manifest("rcommander_ar_tour")
import rospy
import std_msgs.msg as stdm
import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh
import ar_pose.msg as ar_msg
import rcommander_ar_pose.utils as rap
import rcommander_pr2_gui.tf_utils as tfu
import rcommander_web.rcommander_auto_server as ras
import tf

import os
import functools as ft
import cPickle as pk
import os.path as pt
import re
import copy
from threading import RLock
import signal

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

def make_orientation_controls(name):
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

def pose_to_tup(p):
    return [p.position.x, p.position.y, p.position.z], \
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


class TagDatabase:

    def __init__(self):
        self.database = {}
        #self.insert('placeholder')

    ##
    # @param tagid
    # @param location pose stamped in the frame of the tag
    # @param behavior
    # @return None
    def insert(self, tagid, ar_location=DEFAULT_LOC, target_location=DEFAULT_LOC, behavior=None):
        self.database[tagid] = {'ar_location': ar_location, 'target_location': target_location, 'behavior': behavior}

    ##
    # @param tagid
    # @erturn dict with keys 'location', 'behavior'
    def get(self, tagid):
        return self.database[tagid].copy()

    def update_target_location(self, tagid, location):
        self.database[tagid]['target_location'] = location

    def update_behavior(self, tagid, behavior):
        self.database[tagid]['behavior'] = behavior

    def tag_ids(self):
        return self.database.keys()

    def has_id(self, tagid):
        return tagid in self.database.keys()


class MarkerDisplay:
    
    def __init__(self, tagid, server, tag_database, tf_listener):
        self.tagid = tagid

        self.server = server
        self.tag_database = tag_database

        self.make_ar_marker()
        self.has_point = False
        self.tf_listener = tf_listener
        self.menu = None

    def get_id_numb(self):
        idnumb = self.tagid.split('_')
        return int(idnumb[-1])

    def make_ar_marker(self, scale=.2):
        name = 'ar_' + self.tagid
        pose = self.tag_database.get(self.tagid)['ar_location']

        int_marker = interactive_marker(name, (pose[0], (0,0,0,1)), scale)
        int_marker.description = 'Tag #%d' % self.get_id_numb() 
        int_marker.controls += [make_sphere_control(name, int_marker.scale)]
        int_marker.controls[0].markers[0].color = stdm.ColorRGBA(0,1,0,.5)
        self.server.insert(int_marker, self.process_feedback)
        self.ar_marker = int_marker

    def update_ar_marker(self, behavior_name):
        self.server.erase(self.ar_marker.name)
        self.ar_marker.description = ('%s (tag #%d)' % (behavior_name, self.get_id_numb()))
        self.server.insert(self.ar_marker)

    def set_menu(self, menu_handler):
        self.menu = menu_handler
        if not self.has_point:
            return
        else:
            self.server.erase(self.target_marker.name)
            self._make_menu()

    def _make_menu(self):
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = self.tagid + '_menu'
        menu_control.markers.append(copy.deepcopy(self.target_marker.controls[0].markers[0]))
        menu_control.always_visible = True
        self.target_marker.controls.append(copy.deepcopy(menu_control))

        self.server.insert(self.target_marker, self.process_feedback)
        self.menu.apply(self.server, self.target_marker.name)

    def make_target_marker(self, name, pose, scale=.2):
        int_marker = interactive_marker(name, (pose[0], (0,0,0,1)), scale)
        int_marker.header.frame_id = 'map' #self.tagid
        int_marker.description = ''
        int_marker.controls.append(make_sphere_control(name, scale/2.))
        int_marker.controls[0].markers[0].color = stdm.ColorRGBA(1,0,0,.5)
        int_marker.controls += make_orientation_controls(name)

        self.server.insert(int_marker, self.process_feedback)
        self.target_marker = int_marker
        if self.menu != None:
            self._make_menu()

    def toggle_point(self):
        if self.has_point:
            self.server.erase(self.target_marker.name)
        else:
            p_ar = self.tag_database.get(self.tagid)['target_location']
            m_ar = tfu.tf_as_matrix(p_ar)
            map_T_ar = tfu.transform('map', self.tagid, self.tf_listener, t=0)
            p_map = tfu.matrix_as_tf(map_T_ar * m_ar)
            self.make_target_marker('target_' + self.tagid, p_map)
            
        self.server.applyChanges()
        self.has_point = not self.has_point

    def process_feedback(self, feedback):
        name = feedback.marker_name
        ar_match = re.search('^ar_', name)
        target_match = re.search('^target_', name)

        if ar_match != None and feedback.event_type == ims.InteractiveMarkerFeedback.BUTTON_CLICK:
            self.toggle_point()

        if target_match != None and feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            m_ar = tfu.tf_as_matrix(pose_to_tup(feedback.pose))
            ar_T_map = tfu.transform(self.tagid, 'map', self.tf_listener, t=0)
            p_ar = tfu.matrix_as_tf(ar_T_map * m_ar)
            self.tag_database.update_target_location(self.tagid, p_ar)


##
# @param callback is a function: f(menu_item, full_action_path)
def menu_handler_from_action_tree(action_tree, callback):
    handler = mh.MenuHandler()
    menu_handler_from_action_tree_helper(handler, action_tree, callback)
    return handler

# {'path':   full path
#  'actions': [{another folder}, {another folder2}, rcom_file}
def menu_handler_from_action_tree_helper(handler, action_tree, callback, parent=None):
    base_path = action_tree['path']
    for action in action_tree['actions']:
        if isinstance(action, dict):
            submenu = handler.insert(action['path'], parent=parent)
            menu_handler_from_action_tree_helper(handler, action, callback, parent=submenu)
        else:
            action_name = os.path.split(action)[1]
            handler.insert(action_name, parent=parent, callback=ft.partial(callback, action_name, action))

class ARTour:

    def __init__(self, rcommander_files_dir, tag_database_name='ar_tag_database.pkl'):
        self.SERVER_NAME = 'ar_tour'
        self.tag_database_name = tag_database_name
        self.rcommander_files_dir = rcommander_files_dir
        self.ar_lock = RLock()

        #Ros init stuff
        rospy.init_node(self.SERVER_NAME)
        self.broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        #Load the database
        try:
            pickle_file = open(self.tag_database_name, 'r')
            self.tag_database = pk.load(pickle_file)
            pickle_file.close()
            rospy.loginfo('Loaded %s.' % self.tag_database_name)
        except Exception, e:
            self.tag_database = TagDatabase()
            rospy.loginfo('Error loading %s. Creating new database.' % self.tag_database_name)
        self.server = ims.InteractiveMarkerServer(self.SERVER_NAME)
        
        #make the default markers
        self.markers = {}
        for tagid in self.tag_database.tag_ids():
            self.make_marker(tagid)
        self.server.applyChanges()

        #Watch files to generate menus
        self.actions_tree = {}
        self.file_watcher = ras.WatchDirectory(self.rcommander_files_dir, self.main_directory_changed)
        self.main_directory_changed(self.rcommander_files_dir)
    
        #Subscribe to AR Pose
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", ar_msg.ARMarkers, self.ar_marker_cb)
        self.marker_visibility = {}
        print 'Ready!'


    def make_marker(self, tagid):
        if not self.markers.has_key(tagid):
            display = MarkerDisplay(tagid, self.server, self.tag_database, self.tf_listener)
            self.markers[tagid] = display
        else:
            print 'Marker', tagid, 'exists! Ignoring make_marker call.'

    def ar_marker_cb(self, msg):
        #print 'called back'
        self.marker_visibility = {}
        for marker in msg.markers:
            self.marker_visibility['4x4_' + str(marker.id)] = True
        #print 'cb', len(msg.markers)

    def _keep_frames_updated(self):
        #print '<<<<<<<<<<<<<<<'
        #ar_frames = rap.find_ar_frames(self.tf_listener)
        visible = self.marker_visibility.keys()
        #print 'visible', visible
        
        #Publish frames that are not visible
        for tagid in self.tag_database.tag_ids():
            #frameid = '/' + tagid
            if not (tagid in visible):
                position, orientation = self.tag_database.get(tagid)['ar_location']
                self.broadcaster.sendTransform(position, orientation, rospy.Time.now(), tagid, '/map')
                print tagid, ' is not visible. creating transforms.'
        
        #make sure all visible frames are in database
        markers_changed = False
        for tagid in visible:
            try:
                ar_location = self.tf_listener.lookupTransform('map', tagid, rospy.Time(0))
                if not self.tag_database.has_id(tagid):
                    print 'inserted', tagid, 'into database'
                    self.tag_database.insert(tagid, ar_location)
                    self.make_marker(tagid)
                    markers_changed = True
                else:
                    self.tag_database.get(tagid)['ar_location'] = ar_location
            except tf.ExtrapolationException, e:
                print 'exception', e
            except tf.ConnectivityException, e:
                print 'exception', e
            except tf.LookupException, e:
                print 'exception', e.__class__

        if markers_changed:
            self.server.applyChanges()
        #print '>>>>>>>>>>>>>>>'

    def _save_database(self):
        print '< Saving',
        pickle_file = open(self.tag_database_name, 'w')
        pk.dump(self.tag_database, pickle_file)
        pickle_file.close()
        print 'done! >'

    def menu_callback(self, marker_name, menu_item, full_action_path, int_feedback):
        self.ar_lock.acquire()
        print 'menu called back on', marker_name, menu_item, full_action_path, int_feedback
        marker_disp = self.markers[marker_name]
        #Record this in DB
        self.tag_database.update_behavior(marker_disp.tagid, full_action_path)
        #Reset the AR marker
        marker_disp.update_ar_marker(menu_item)
        self.server.applyChanges()
        self.ar_lock.release()

    def main_directory_changed(self, main_path_name):
        self.ar_lock.acquire()
        rospy.loginfo('main_directory_changed: rescanning ' + self.rcommander_files_dir)
        self.actions_tree = ras.find_all_actions(self.rcommander_files_dir)
        rospy.loginfo('All actions found\n %s \n' % str(self.actions_tree))
        for marker_name in self.markers.keys():
            menu_handler = menu_handler_from_action_tree(self.actions_tree, ft.partial(self.menu_callback, marker_name))
            self.markers[marker_name].set_menu(menu_handler)
        self.server.applyChanges()
        self.ar_lock.release()

    def step(self, timer_obj):
        self.ar_lock.acquire()
        self._keep_frames_updated()
        if (self.i % (5*5) == 0):
            self._save_database()
        self.i += 1
        self.ar_lock.release()

    def run(self):
        self.i = 0
        rospy.Timer(rospy.Duration(2.), self.step)


if __name__=="__main__":
    import sys
    from PyQt4 import QtCore#, QtGui

    def sigint_handler(*args):
        QtCore.QCoreApplication.quit()

    #signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGINT, sigint_handler)
    app = QtCore.QCoreApplication(sys.argv)
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    artour = ARTour(sys.argv[1], sys.argv[2])
    artour.run()
    rospy.loginfo('RCommander AR Tour Server up!')
    app.exec_()


#####
# How to play back behaviors?
# rviz
#
# browser
#
# rcommander_web
#
# rcommander_auto_server
#
# rcommander_ar_tour

#
# browser => auto_server (don't have 3D point)
#
# browser => rcommander_ar_tour (don't have the ability to execute yet)
#
# special brower => special rcommander_ar_tour (has 3D and execution)
#
# How would rcommander web integrate with auto server?
#
# Rcommander auto server
#
# 1) complete separation of web and ar_tour
#       web for list of behaviors
#       tour for ar toolkit type things
#       problem of two behaviors trying to execute at the same time.  bad!
#
# 2) ar tour sets the parameters
#       web activates/replay everything
#       
#       Web has to call auto server vs ar_tour accordingly
#       auto_server/ar_tour appears as menus in web interface?
#
#       construct a new auto_server/ar_tour mashup
#





############################
# Next imitate rcommander_web's loading of behaviors in a directory!

################
# A main interactive marker for the TAG itself
#   Tag initialization routine
#       Everytime we see a tag in the map, store its location, average all known detections.
#       {'tag_name': {'location': list of poses, 'behavior': 'folder location'}}
#
#
#
#   Need a way to initialize tags.
#       Drive PR2 through house
#   From the database, create all known tags
#   On sensing create a 
#       
#   A menu marker class monitors the state of a directory








    ##self.create_menu(name, self.server, int_marker, int_marker.controls[0].markers[0])
    #def create_menu(self, name, server, int_marker, marker):
    #    menu_control = ims.InteractiveMarkerControl()
    #    menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
    #    menu_control.description=""
    #    menu_control.name = name + '_menu'
    #    menu_control.markers.append(copy.deepcopy(marker))
    #    menu_control.always_visible = True
    #    int_marker.controls.append(copy.deepcopy(menu_control))

    #    #menu_handler = mh.MenuHandler()
    #    #menu_handler.insert("First Entry", callback=self.process_feedback)
    #    #menu_handler.insert("Second Entry", callback=self.process_feedback)
    #    #sub_menu_handle = menu_handler.insert("Submenu")
    #    #menu_handler.insert("First Entry", parent=sub_menu_handle, callback=self.process_feedback)
    #    #menu_handler.insert("Second Entry", parent=sub_menu_handle, callback=self.process_feedback)
    #    menu_handler.apply(server, int_marker.name)




        #r = rospy.Rate(2)
        #print 'Ready.'
        #while not rospy.is_shutdown():
        #r.sleep()
        #rospy.spin()




#
#   A 
 

#class BehaviorMenu:
#
#    def __init__(self, name, pose, scale, server):
#        self.server = server
#        self.create_marker(name, pose, scale)
#        self.create_menu()
#
#    def create_marker(self, name, pose, scale):
#        int_marker = interactive_marker(name, pose, scale)
#
#        #int_marker = ims.InteractiveMarker()
#        #int_marker.header.frame_id = "/map"
#        #int_marker.pose.position.y = -3.0
#        #int_marker.scale = 1
#        #int_marker.name = "context_menu"
#        #int_marker.description = ""
#
#        ## make one control using default visuals
#        #control = ims.InteractiveMarkerControl()
#        #control.interaction_mode = ims.InteractiveMarkerControl.MENU
#        #control.description=""
#        #control.name = 
#        #"menu_only_control"
#        #int_marker.controls.append(copy.deepcopy(control))
#
#        # make one control showing a box
#        #marker = make_box(int_marker)
#        #control.markers.append(marker)
#        #control.always_visible = True
#        #int_marker.controls.append(control)
#        #self.int_marker = int_marker
#
#
#    def create_menu(self):
#        menu_handler = mh.MenuHandler()
#        menu_handler.insert( "First Entry", callback=self.process_feedback )
#        menu_handler.insert( "Second Entry", callback=self.process_feedback )
#        sub_menu_handle = menu_handler.insert( "Submenu" )
#        menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=self.process_feedback )
#        menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=self.process_feedback )
#        self.server.insert(self.int_marker, self.process_feedback)
#        menu_handler.apply(self.server, self.int_marker.name )
#
#    def create_menu2(self):
#        menu_handler = mh.MenuHandler()
#        menu_handler.insert( "First Entry", callback=self.process_feedback )
#        sub_menu_handle = menu_handler.insert( "Submenu" )
#        menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=self.process_feedback )
#        self.server.insert(self.int_marker, self.process_feedback)
#        menu_handler.apply(self.server, self.int_marker.name )
#
#
#    def process_feedback(self, feedback):
#        self.server.erase('context_menu')
#        self.create_marker()
#        self.create_menu2()
#
