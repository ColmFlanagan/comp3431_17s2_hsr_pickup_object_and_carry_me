#!/usr/bin/env python
from __future__ import division
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from sets import Set
import numpy as np
import roslaunch
import math
import tf
import geometry_msgs.msg 
import time
from myvis.msg import Object
from myvis.msg import Objects
from tmc_darknet_msgs.msg import Detection
from tmc_darknet_msgs.msg import Detections
DETECTED = "/yolo2_node/detections"
DEPTH_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
RGB_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
MAP = "/map"
CAMERA = "/head_rgbd_sensor_rgb_frame"

class image_converter:

	def __init__(self):
		rospy.init_node('image_converter', anonymous=True)
		self.tfe = tf.TransformListener()
		self.beacon_pub = rospy.Publisher("Detected_Object", Marker, queue_size=10)
		self.depthimage = np.zeros((480,640,1), np.uint8)
		self.colorimage = np.zeros((480,640,3), np.uint8)
		self.marked = Set()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(RGB_IMAGE_TOPIC, Image, self.callback, queue_size=1, buff_size=480*640*8)
		self.depth_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.get_depth)
		self.depth_sub = rospy.Subscriber(DETECTED, Detections, self.get_detection)
		self.puba = rospy.Publisher("Objects", Objects, queue_size=10)
		self.pub2 = rospy.Publisher("ObjectsMap", Objects, queue_size=10)
		# Initialise the depth image 
	
	def get_detection(self, data):
		print("get detection")

		try:
			t = self.tfe.getLatestCommonTime(CAMERA, MAP)
			position, quaternion = self.tfe.lookupTransform(MAP, CAMERA,  t)
			print(position)

	
			obsArrayMap = Objects()
			obsArray = Objects()
			print("found")
			for xa in range(0, len(data.detections)):
				y = data.detections[xa].y
				h = data.detections[xa].height 
				x = data.detections[xa].x 
				w = data.detections[xa].width 
				cropped_img = self.depthimage[y-h/3:y+h/3, x-w/3:x+w/3]
		
				#cimg = self.colorimage[392:560, 424:761]
				cimg = self.colorimage[y-h/2:y+h/2, x-w/2:x+w/2]
				#if(data.detections[xa].class_name == "bottle"):
				#	cv2.imshow('img',cimg)
			#		cv2.waitKey(0)
				cropped_img.setflags(write=1)
				deptharray = np.asarray(cropped_img)
				#nana = np.isnan(deptharray)
				#deptharray[nana] = 0
				depth = np.nanmean(deptharray)

				#a = 0.0015
				#b= 0.0015
				a = 0.0012
				Xa=(x-320)*a*depth
				Ya=(y-240)*a*depth

				ObjectA = Object()
				ObjectA.name = data.detections[xa].class_name
				ObjectA.x= Xa
				ObjectA.y= Ya
				ObjectA.z = depth
				ObjectA.camera_x= x
				ObjectA.camera_y = y
				
				
				obsArray.Objects.append(ObjectA)
			obsArray.length = len(data.detections)
			self.puba.publish(obsArray)
		
			for xb in range(0, len(obsArray.Objects)):

				ObjectB = Object()
				ObjectB.name = obsArray.Objects[xb].name
				ObjectB.x= position[0]+obsArray.Objects[xb].z
				ObjectB.y= position[1]-obsArray.Objects[xb].x
				ObjectB.z = position[2]-obsArray.Objects[xb].y
				print("		X="+str(obsArray.Objects[xb].x))
				print("		Y="+str(obsArray.Objects[xb].y))
				print("		Z="+str(obsArray.Objects[xb].z))

				obsArrayMap.Objects.append(ObjectB)
				print("		")
				if(obsArray.Objects[xb].name == "bottle" or obsArray.Objects[xb].name == "cup"):
					marker = Marker()
					marker.id = xa
					marker.header.frame_id = MAP
					marker.type = marker.SPHERE
					marker.action = marker.ADD
					marker.scale.x = 0.05
					marker.scale.y = 0.05
					marker.scale.z = 0.05
					marker.color.a = 1.0
					marker.color.r = 0
					marker.color.g = 0
					marker.color.b = 0
					marker.pose.orientation.x = quaternion[0]
					marker.pose.orientation.y = quaternion[1]
					marker.pose.orientation.z = quaternion[2]
					marker.pose.orientation.w = quaternion[3]
					marker.pose.position.x = -position[0]+obsArray.Objects[xb].z
					marker.pose.position.y = position[1]-obsArray.Objects[xb].x
					marker.pose.position.z = position[2]-obsArray.Objects[xb].y
					print("		"+(obsArray.Objects[xb].name))
					print("		X="+str(position[0]))
					print("		Y="+str(position[1]))
					print("		Z="+str(position[2]))
					print("		"+(obsArray.Objects[xb].name))
					print("		X="+str(obsArray.Objects[xb].z))
					print("		Y="+str(obsArray.Objects[xb].x))
					print("		Z="+str(obsArray.Objects[xb].y))
					print("		"+(obsArray.Objects[xb].name))
					print("		X="+str(marker.pose.position.x))
					print("		Y="+str(marker.pose.position.y))
					print("		Z="+str(marker.pose.position.z))
					self.beacon_pub.publish(marker)
			obsArrayMap.length = obsArray.length
			self.pub2.publish(obsArrayMap)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("failed to get pose")

	def get_depth(self, data):

		try:
			
			cv_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			self.depthimage = cv_image
		except CvBridgeError as e:
			print(e)	
		sys.exit()

	def callback(self, data):
		y = 10
		h = 100
		x = 10
		w = 100
		self.colorimage = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cropped_img = self.depthimage[y:y+h, x:x+w]
		depth = np.nanmean(np.asarray(cropped_img))

		#print(depth)


def main(args):
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
