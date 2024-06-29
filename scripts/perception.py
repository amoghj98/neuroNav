#!/usr/bin/env python

import rospy
import rospkg
import time
import sys
import numpy as np

from math import *
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Header
from nav_msgs.msg import Odometry
import tf
import os


class perception(object):
	def __init__(self):
		rospy.init_node('perception', log_level = rospy.DEBUG)
		self.init_params()
		self._goalPtPub = rospy.Publisher('/perception/goal_point/', Vector3, queue_size = 10)
		rospy.Subscriber("/vrpn_client_node/ring/pose", PoseStamped, self._ringPoseCB)
		self._goalPtTimer = rospy.Timer(rospy.Duration(0, (1e9/50)), self._goalptPub)

	def init_params(self):
		self.x0 = -1
		self.v0 = 0.1
		self.t0 = 0
		self.ringPose = np.zeros((3,1), dtype=np.float32)
		self.ringOri = np.zeros((3,1), dtype=np.float32)
		self._goalPt = Vector3()
		self._goalPt.x = self._goalPt.y = self._goalPt.z = 0

	def _ringPoseCB(self, data) :
		pose = data.pose.position
		self.ringPose[0] = pose.x
		self.ringPose[1] = pose.y
		self.ringPose[2] = pose.z
		ori = data.pose.orientation
		self.ringOri = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
		#
		t1 = data.header.stamp.secs + (data.header.stamp.nsecs/1e9)
		self.v0 = (self.ringPose[0] - self.x0)/(t1 - self.t0)
		self.x0 = self.ringPose[0]
		self.t0 = t1
		#
		self._goalPt.y = self.ringPose[1]

	def _goalptPub(self, _te):
		self._goalPtPub.publish(self._goalPt)

	def LPE(self):
		den = np.array([(5-self.v0), (5+self.v0)]).reshape(2,1)
		num = np.array([[(8.5+self.x0), ((5*self.x0)+(8.5*self.v0))], [(8.5-self.x0), ((5*self.x0)+(8.5*self.v0))]]).reshape(2,2)
		soln = num/den
		idx = np.where(soln[:, 0]>1.7)
		soln = soln[idx].squeeze()
		# soln = soln
		# print(soln.shape)
		# print(soln)
		self._goalPt.x = soln[1] + 0.3



if __name__ == "__main__":
	perceptor = perception()
	rospy.loginfo("Publishing goal points!")
	# perceptor.LPE()
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		perceptor.LPE()
		rate.sleep()
	rospy.signal_shutdown(0)