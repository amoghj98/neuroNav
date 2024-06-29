#!/usr/bin/env python

import rospy
import rospkg
import time
import sys
import numpy as np

from math import *
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import tf
import os

from pid import *
from bebop import *
	

if __name__ == '__main__':
	savePath = os.path.join('/home/amogh/ws/src/waypoint_manager/')
	xPos, yPos, zPos = [], [], []
	xOri, yOri, zOri = [], [], []
	robot = bebop(nSetPt=3)
	# rospy.init_node('waypoint_control', anonymous=True)
	# rospy.sleep(1.0)
	os.system("rostopic pub --once bebop/takeoff std_msgs/Empty")
	# os.system("rostopic pub --once bebop/reset std_msgs/Empty")	# emergency kill - will kill drone in mid-air. DO NOT USE
	while not rospy.is_shutdown():
		if robot._ready:
			rospy.logwarn("Drone ready to manoeuver")
			robot.executeManoeuver()
			break
	rospy.loginfo(robot.t2-robot.t1)
	rospy.loginfo(robot.t3-robot.t1)
	os.system("rostopic pub --once bebop/land std_msgs/Empty")
	rospy.loginfo("Controller Node Has Shutdown.")
	np.save(savePath+'xPos', np.asarray(xPos))
	np.save(savePath+'yPos', np.asarray(yPos))
	np.save(savePath+'zPos', np.asarray(zPos))
	np.save(savePath+'xOri', np.asarray(xOri))
	np.save(savePath+'yOri', np.asarray(yOri))
	np.save(savePath+'zOri', np.asarray(zOri))
	rospy.signal_shutdown(0)
	# sys.exit(0)
	# robot.executeManoeuver()
	
