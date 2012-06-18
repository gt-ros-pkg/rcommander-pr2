import roslib; roslib.load_manifest("interactive_markers")
import rospy
import interactive_markers.interactive_marker_server as ims
import std_msgs.msg as stdm
import interactive_markers.menu_handler as mh
import re
import copy

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
        self.insert('placeholder')

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

    def tag_ids(self):
        return self.database.keys()


class MarkerDisplay:
    
    def __init__(self, tagid, server, tag_database):
        self.tagid = tagid

        self.server = server
        self.tag_database = tag_database

        #self.make_target_marker('target_' + self.tagid, self.tag_database.get(self.tagid)['target_location'], .5)
        self.make_ar_marker('ar_' + self.tagid, self.tag_database.get(self.tagid)['ar_location'], .5)
        self.has_point = False

        #self.behavior_menu = BehaviorMenu('menu_' + self.tag_id, self.server)

    def make_ar_marker(self, name, pose, scale):
        int_marker = interactive_marker(name, pose, scale)
        int_marker.controls += [make_sphere_control(name, int_marker.scale)]
        int_marker.controls[0].markers[0].color = stdm.ColorRGBA(0,1,0,.5)
        self.server.insert(int_marker, self.process_feedback)
        self.ar_marker = int_marker

    def create_menu(self, name, server, int_marker, marker):
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.description=""
        menu_control.name = name + '_menu'
        menu_control.markers.append(copy.deepcopy(marker))
        menu_control.always_visible = True
        int_marker.controls.append(copy.deepcopy(menu_control))

        menu_handler = mh.MenuHandler()
        menu_handler.insert( "First Entry", callback=self.process_feedback )
        menu_handler.insert( "Second Entry", callback=self.process_feedback )
        sub_menu_handle = menu_handler.insert( "Submenu" )
        menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=self.process_feedback )
        menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=self.process_feedback )
        menu_handler.apply(server, int_marker.name)

    def make_target_marker(self, name, pose, scale):
        int_marker = interactive_marker(name, pose, scale)
        int_marker.description = ''
        int_marker.controls.append(make_sphere_control(name, scale/2.))
        int_marker.controls[0].markers[0].color = stdm.ColorRGBA(1,0,0,.5)
        int_marker.controls += make_orientation_controls(name)

        self.server.insert(int_marker, self.process_feedback)
        self.create_menu(name, self.server, int_marker, int_marker.controls[0].markers[0])
        self.target_marker = int_marker

    def toggle_point(self):
        if self.has_point:
            self.server.erase(self.target_marker.name)
        else:
            self.make_target_marker('target_' + self.tagid, self.tag_database.get(self.tagid)['target_location'], .5)
            
        self.server.applyChanges()
        self.has_point = not self.has_point

    def process_feedback(self, feedback):
        name = feedback.marker_name
        ar_match = re.search('^ar_', name)
        target_match = re.search('^target_', name)

        if ar_match != None and feedback.event_type == ims.InteractiveMarkerFeedback.BUTTON_CLICK:
            self.toggle_point()

        if target_match != None and feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            self.tag_database.update_target_location(self.tagid, pose_to_tup(feedback.pose))



class ARTour:

    def __init__(self):
        self.SERVER_NAME = 'ar_tour'
        self.tag_database = TagDatabase()
        print 'initiating tag database'
        rospy.init_node(self.SERVER_NAME)
        self.server = ims.InteractiveMarkerServer(self.SERVER_NAME)

        self.markers = {}
        for tagid in self.tag_database.tag_ids():
            display = MarkerDisplay(tagid, self.server, self.tag_database)
            self.markers[tagid] = display
            #[self.server.insert(m, display.process_feedback) for m in display.get_markers()]

        #self.bmenu = BehaviorMenu(self.server)
        self.server.applyChanges()

    def run(self):
        print 'ready.'
        rospy.spin()



if __name__=="__main__":
    a = ARTour()
    a.run()

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
