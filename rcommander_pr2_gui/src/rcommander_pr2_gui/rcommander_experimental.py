#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander_core.rcommander as rc
import pr2_utils as pu
import rospy
import tf 

rospy.init_node('rcommander', anonymous=True)
tf = tf.TransformListener()
#pr2 = pu.PR2(tf)
pr2 = None
#print 'RCOMMANDER EXP IS SHUTDOWN', rospy.is_shutdown()
rc.run(pr2, tf, ['experimental'])
