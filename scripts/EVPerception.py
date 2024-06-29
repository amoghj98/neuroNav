#!/usr/bin/env python

import cv2
import numpy as np
from signal import signal, SIGINT
#import ros_numpy
import rospy
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
import argparse
from argparse import RawTextHelpFormatter
import torch
import torch.nn as nn
from matplotlib import pyplot as plt
import math

from dvs.DAVIS346 import DAVIS346
# from pyaer import libcaer
# from pyaer.davis import DAVIS

import expUtils as util
# import pyrealsense2 as rs


class EVPerception(object):
	def __init__(self, f=50, render=False):
		self.f = f
		self.render = render
		self.Tvec = np.zeros((3,1), dtype=np.float32)	# camera pose
		self.Rvec = np.zeros((3,1), dtype=np.float32)	# camera ori
		rospy.init_node('perception', log_level = rospy.DEBUG)
		self.init_params()
		rospy.Subscriber("/vrpn_client_node/camera/pose", PoseStamped, self._camPoseCB)
		rospy.Subscriber("/vrpn_client_node/ring/pose", PoseStamped, self._ringPoseCB)
		self._goalPtPub = rospy.Publisher('/perception/goal_point/', Vector3, queue_size = 10)
		self._goalPtTimer = rospy.Timer(rospy.Duration(0, (1e9/self.f)), self._goalptPub)
		self.spikecam = DAVIS346(packet_interval=int(1e6/self.f))

	def init_params(self):
		self.depth = 1
		self.ringPose = np.zeros((3,1), dtype=np.float32)
		self.ringOri = np.zeros((3,1), dtype=np.float32)
		self._goalPt = Vector3()
		self._goalPt.x = self._goalPt.y = self._goalPt.z = 0
		rot = {'x':0, 'y':0, 'z':0}
		self.R = util.getRotMat(rot)
		self.K = np.array(([[247.069605, 0, (346/2)], [0, 247.069605, (260/2)], [0, 0, 1]]))

	def _camPoseCB(self, data):
		pose = data.pose.position
		self.Tvec[0] = pose.x
		self.Tvec[1] = pose.y
		self.Tvec[2] = pose.z
		ori = data.pose.orientation
		self.Rvec = np.array((180, 0, 0)) * (np.pi/180)
		self.r = self._euler_from_quaternion(ori)
		rot = {'x':self.r[0], 'y':self.r[1], 'z':self.r[2]}
		self.R = util.getRotMat(rot)

	def _ringPoseCB(self, data):
		pose = data.pose.position
		self.ringPose[0] = pose.x
		self.ringPose[1] = pose.y
		self.ringPose[2] = pose.z
		self.depth = self.Tvec[2] - self.ringPose[2]
		ori = data.pose.orientation
		self.ringOri = self._euler_from_quaternion(ori)

	def _goalptPub(self, _te):
		self._goalPtPub.publish(self._goalPt)

	def _euler_from_quaternion(self, q, returnDegrees=False):
		x = q.x
		y = q.y
		z = q.z
		w = q.w
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		if returnDegrees:
			return np.array((roll_x, pitch_y, yaw_z)) * (180/np.pi)
		else:
			return np.array((roll_x, pitch_y, yaw_z)) # in radians

	def circleDetector(self, data):
		#Get binary mask [0, 255]
		data[data<255] = 0
		data[data==255] = 255
		data = np.array(data, dtype=np.uint8)
		img = cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)
		# Morpological Ops
		k = 5
		j = cv2.GaussianBlur(data, (k,k), 0)
		# thresholding
		j[j>50] = 255 
		# Hough transform
		circles = cv2.HoughCircles(j, cv2.HOUGH_GRADIENT, 1, 1000, param1=255, param2=0.95, minRadius=32, maxRadius=43)
		if circles is not None:
			circles = np.uint16(np.around(circles))
			for k in circles[0,:]:
				center = (k[0], k[1])
				radius = k[2]
				# draw the outer circle
				cv2.circle(img, center, radius,(0,255,0), 3)
				# draw the center of the circle
				cv2.circle(img, center, 2, (0,0,255), 3)
		if self.render:
			cv2.imshow('raw_events', data)
			cv2.imshow('event_frame_filtered', j)
			cv2.imshow('detected circles', img)
			cv2.waitKey(1)
		if circles is not None:
			return center
		else:
			return (0, 0)

	def predict(self):
		data = self.spikecam.get_packet()
		if data is not None:
			img = self.getEventData(data)
			center = np.array(self.circleDetector(img))[np.newaxis, :, np.newaxis]
			goalPt = util.inverseHomographicTransform(center, Tvec=self.Tvec, Rvec=self.Rvec, K=self.K, R=self.R, depth=self.depth).squeeze()[:, np.newaxis]
			goalPt = util.evPlanner(goalPt)
			# print(goalPt - self.ringPose)
		self._goalPt.x = goalPt[0]
		self._goalPt.y = goalPt[1]
		self._goalPt.z = goalPt[2]

	def getEventData(self, data):
		(pol_events, num_pol_event, special_events, num_special_event, frames_ts, frames, imu_events, num_imu_event) = data
		spike_img = np.full((self.spikecam.davis346.dvs_size_Y, self.spikecam.davis346.dvs_size_X), 0, dtype=np.uint8)
		spike_inp = np.full((self.spikecam.davis346.dvs_size_Y, self.spikecam.davis346.dvs_size_X), 0, dtype=np.uint8)
		if pol_events is not None:
			spike_img[pol_events[:,2], pol_events[:,1]] = 255
			spike_inp[pol_events[:,2], pol_events[:,1]] += 1
		with torch.no_grad():
			input = torch.tensor(spike_inp).float()
			input = input[None,None,:]
			visual_frame = np.array(input[0,0,:,:])
			visual_frame = ((visual_frame - visual_frame.min()) * (1/(visual_frame.max() - visual_frame.min()) * 255)).astype('uint8')
			imgin = np.array(visual_frame, dtype=np.uint8)
			return imgin


if __name__ == '__main__':
	desc = (
		"Script to generate dataset from a MAVLINK-Gazebo simulation.\nWARNING: The simulation must be running BEFORE this script is executed. Executing this script without the simulation running may lead to unpredictable behaviour.\nIn case the given HDF5 file already contains a group with name 'groupname', this script will fail gracefully, and report the corresponding error. Existing groups can be viewed/deleted using the script 'deleter.py'")
	epilog = ("For further documentation, refer the Dataset Generation Suite documentation page at https://github.com/")
	parser = argparse.ArgumentParser(description=desc, epilog=epilog, formatter_class=RawTextHelpFormatter)
	args = parser.parse_args()
	perceptor = EVPerception(render=True)
	rospy.loginfo("Publishing goal points!")
	# rospy.sleep(1)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		perceptor.predict()
		rate.sleep()
	rospy.signal_shutdown(0)
