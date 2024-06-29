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


class bebop(object):
	def __init__(self, f=50, nSetPt=3) :
		rospy.init_node('waypoint_control', log_level = rospy.DEBUG)
		self.wp = None
		self.init_params(nSetPt)
		self._ready = False
		self._done = False
		self._wpIdx = 0
		self._lastWp = False
		self.freq = f
		self.initPose = None
		self._yawCtrler = pid(k=np.array([[0.8, 0.0, -0.32]]))
		self._altCtrler = pid(k=np.array([[0.4, 0.0, -0.08]]), setpt=np.array([self.wp[self._wpIdx][1]]))
		self._zCtrler = pid(k=np.array([[0.4, 0.0, -0.32]]), setpt=np.array([self.wp[self._wpIdx][2]]))
		self._xCtrler = pid(k=np.array([[0.4, 0.0, -0.32]]), setpt=np.array([self.wp[self._wpIdx][0]]))
		self._velPub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 10)
		rospy.Subscriber("/vrpn_client_node/parrotBebop/pose", PoseStamped, self._dronePoseCB)
		rospy.Subscriber("/bebop/odom", Odometry, self._velCB)
		rospy.Subscriber('/perception/goal_point/', Vector3, self._goalPtCB)
		rospy.Timer(rospy.Duration(0, (1e9/(self.freq))), self._failsafe)
		self._stabiliserTimer = rospy.Timer(rospy.Duration(0, (1e9/(self.freq))), self._stabiliser)
		self._setptTimer = rospy.Timer(rospy.Duration(0, (1e9/(self.freq))), self._setptPub)

	def init_params(self, n) :
		#### Read Global Parameters from the launch file ####
		for i in range(1, 1+n):
			w_x = rospy.get_param('/w'+str(i)+'_x')
			w_y = rospy.get_param('/w'+str(i)+'_y')
			w_z = rospy.get_param('/w'+str(i)+'_z')
			if self.wp is None:
				self.wp = np.array([w_x, w_y, w_z]).reshape(1, 3)
			else:
				self.wp = np.append(self.wp, np.array([w_x, w_y, w_z]).reshape(1, 3), axis=0)
		print(self.wp)
		# sys.exit(0)
		# self.wp1 = np.append(np.array([w1_x, w1_y, w1_z]).reshape(1, 3), axis=0)
		self.t_target = rospy.get_param('/t_target')
		# np arrs
		self.dronePose = np.zeros((3,1), dtype=np.float32)
		self.droneOri = self.ringOri = np.zeros((3,1), dtype=np.float32)
		self.droneVel = self.droneOmega = np.zeros((3,1), dtype=np.float32)
		# gal point
		self.goalPt = Vector3()
		# setpt msg
		self._cmd = Twist()
		self._cmd.linear.x  = 0
		self._cmd.linear.y = 0
		self._cmd.linear.z = 0
		self._cmd.angular.x = 0
		self._cmd.angular.y = 0
		self._cmd.angular.z = 0

	def _setptPub(self, _te):
		self._velPub.publish(self._cmd)

	def _dronePoseCB(self, data) :
		pose = data.pose.position
		self.dronePose[0] = pose.x
		self.dronePose[1] = pose.y
		self.dronePose[2] = pose.z
		# xPos.append(pose.x)
		# yPos.append(pose.y)
		# zPos.append(pose.z)
		if self.initPose is None:
			self.initPose = self.dronePose
		ori = data.pose.orientation
		self.droneOri = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
		# xOri.append(self.droneOri[0])
		# yOri.append(self.droneOri[2])
		# zOri.append(self.droneOri[1])
		# self.droneOri = self.droneOri * (np.pi/180)

	def _velCB(self, data) :
		lin = data.twist.twist.linear
		ang = data.twist.twist.angular
		self.droneVel = np.array([lin.x, lin.y, lin.z])
		self.droneOmega = np.array([ang.x, ang.y, ang.z])

	def _goalPtCB(self, data):
		self.goalPt = data

	def _stabiliser(self, _te):
		# yaw
		self._cmd.angular.z = self._yawCtrler.output(self.droneOri[1])
		# alt stabilisation
		self._cmd.linear.z = self._altCtrler.output(self.dronePose[1])
		# stablise x-direction
		self._cmd.linear.y = -self._xCtrler.output(self.dronePose[0])
		# stablise x-direction
		self._cmd.linear.x = -self._zCtrler.output(self.dronePose[2])
		#
		if self._ready:
			if self._wpIdx==0:
				self.t1 = rospy.Time.now()
			if (np.absolute(self.wp[self._wpIdx][2] - self.dronePose[2]) < 0.25) and (np.absolute(self.wp[self._wpIdx][0] - self.dronePose[0]) < 0.2):
				rospy.loginfo("Reached waypoint %s/%s", str(self._wpIdx + 1), str(self.wp.shape[0]))
				if self._lastWp:
					self._done = True
					self.t3 = rospy.Time.now()
				else:
					if self._wpIdx==1:
						self.t2 = rospy.Time.now()
					self._wpIdx += 1
					self._altCtrler.set_setpt(setpt=np.array([self.wp[self._wpIdx][1]]))
					self._zCtrler.set_setpt(setpt=np.array([self.wp[self._wpIdx][2]]))
					self._xCtrler.set_setpt(setpt=np.array([self.wp[self._wpIdx][0]]))
					if self._wpIdx == self.wp.shape[0] - 1:
						self._lastWp = True

	def _shutdownTimers(self):
		self._stabiliserTimer.shutdown()
		self._setptTimer.shutdown()

	def _failsafe(self, _te) :
		"""
			Failsafe conditions:
				AltFail: zn > threshold
				VelFail: |vx| or |vy| or |vz| > threshold
				AttitudeFail: |R| or |P| or |Y| > threshold
				AngRateFail: |Wr| or |Wp| or |Wy| > threshold
		"""
		failed = False
		altThresh = 1.4
		attThresh = np.array([20, 15, 30]) * (180/np.pi)
		velThresh = np.array([1.5, 1.5, 1.5])
		fence = np.array([1.9, 1.5, 1.9]).reshape(3,1)
		if self.dronePose[1] > altThresh:
			failed = True
			rospy.logwarn("WARNING: FAILSAFE TRIGGERED: AltFail")
		if np.any(np.absolute(self.droneOri) > attThresh):
			failed = True
			rospy.logwarn("WARNING: FAILSAFE TRIGGERED: AttFail")
		if np.any(np.absolute(self.droneVel) > velThresh):
			failed = True
			rospy.logwarn("WARNING: FAILSAFE TRIGGERED: VelFail")
		if np.any(np.absolute(self.dronePose) > fence):
			failed = True
			rospy.logwarn("WARNING: FAILSAFE TRIGGERED: FenseBreach")
		if failed:
			self._shutdownTimers()
			self._cmd.linear.x  = 0
			self._cmd.linear.y = 0
			self._cmd.linear.z = 0
			self._cmd.angular.x = 0
			self._cmd.angular.y = 0
			self._cmd.angular.z = 0
			self._velPub.publish(self._cmd)
			os.system("rostopic pub --once bebop/land std_msgs/Empty")
			rospy.sleep(4.0)
			sys.exit(0)
		if (np.absolute(self.dronePose[1] - self.wp[0][1]) < 10e-2) and (np.absolute(self.droneOri[1]) < 5e-3) and (np.absolute(self.dronePose[0] - self.wp[0][0]) < 10e-2):
			self._ready = True
			# self.wp[1, 0] = round(self.goalPt.x, 2)
			# self.wp[1, 1] = round(self.goalPt.y, 2)
			# self.wp[1, 2] = round(self.goalPt.z, 2)
			# rospy.loginfo(self.wp)
			# if self.wp[1, 0] > 0:
			# 	self.wp[1, 0] -= 1
			# else:
			# 	self.wp[1, 0] += 1
			# self._shutdownTimers()
			# os.system("rostopic pub --once bebop/land std_msgs/Empty")

	def executeManoeuver(self) :
		rate = rospy.Rate(self.freq)
		while not rospy.is_shutdown():
			while not self._ready:
				pass
			# xzTimer = rospy.Timer(rospy.Duration(0, (1e9/(self.freq))), self._xzStabiliser)
			if self._done:
				self._shutdownTimers()
				break
			rate.sleep()
		self._cmd.linear.x  = 0
		self._cmd.linear.y  = 0
		self._cmd.angular.x  = 0
		self._cmd.angular.y  = 0
	

if __name__ == '__main__':
	robot = bebop()
	debug = False
	while debug:
		robot.debug()
		rospy.sleep(1.0)
	# rospy.init_node('waypoint_control', anonymous=True)
	rospy.sleep(1.0)
	os.system("rostopic pub --once bebop/takeoff std_msgs/Empty")
	while not rospy.is_shutdown():
		# print(robot.droneOri)
		pass
	# robot.executeManoeuver()
	
