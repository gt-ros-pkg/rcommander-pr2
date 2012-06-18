import roslib; roslib.load_manifest('rcommander_web')
import rospy
from rcommander_web.srv import ActionInfo, ActionInfoResponse, ActionInfoRequest


rospy.init_node('get_actions', anonymous=True)
list_actions = rospy.ServiceProxy('list_rcommander_actions', ActionInfo)
print 'got\n', list_actions('.')
print 'got\n', list_actions('test_behaviors')
print 'got\n', list_actions('test_behaviors/subdir')
print 'got\n', list_actions('test_behaviors/subdir/subdir')
