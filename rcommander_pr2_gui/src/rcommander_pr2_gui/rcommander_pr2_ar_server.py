#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2_gui')
#import roslib; roslib.load_manifest("rcommander_ar_tour")
import pr2_utils as pu
import rcommander_ar_tour.rcommander_ar_server as rcs
import sys
import tf 
import rospy

rospy.init_node('rcommander_pr2_server')
path_to_rcommander_files = sys.argv[1]
path_to_tag_database = sys.argv[2]
tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rcs.run(pr2, tf, path_to_rcommander_files, path_to_tag_database)
