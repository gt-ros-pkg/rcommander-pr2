#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rcommander.rcommander as rc
import pypr2.pr2_utils as pu
import rospy
import tf 

rospy.init_node('rcommander', anonymous=True)
tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rc.run_rcommander(['default', 'default_frame', 'pr2'], pr2, tf)
