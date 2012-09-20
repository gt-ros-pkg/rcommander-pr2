import roslib; roslib.load_manifest('rcommander_web')
import rcommander_web.rcommander_auto_server as rcs
import sys
import rospy

class MyRobotClass:
    def __init__(self):
        self.some_resource = "hello"

rospy.init_node('rcommander_web_test')
path = sys.argv[1]
robot = MyRobotClass()
rcs.run(robot, path)
