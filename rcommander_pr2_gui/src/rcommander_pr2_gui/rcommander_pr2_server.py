#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2_gui')
import pypr2.pr2_utils as pu
import rcommander_web.rcommander_auto_server as rcs
import sys
import tf 
import rospy

rospy.init_node('rcommander_pr2_server')
path = sys.argv[1]
tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rcs.run(pr2, path)
