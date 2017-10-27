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
import image_geometry
import time
from myvis.msg import Object
from myvis.msg import Objects
from tmc_darknet_msgs.msg import Detection
from tmc_darknet_msgs.msg import Detections
from sensor_msgs.msg import CameraInfo
DETECTED = "/yolo2_node/detections"
DEPTH_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
RGB_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
MAP = "/map"
CAMERA = "/head_rgbd_sensor_rgb_frame"
FACTOR2 = 0.0012
FACTOR1 = 0.0025
CAMERA_TOPIC = "/hsrb/head_rgbd_sensor/rgb/camera_info"
#OBJECT_TO_PICK = 'sports ball'
#OBJECT_TO_PICK = 'bottle'
DETECTED_OBS = "Detected_Object"
TOPICK_NODE = "topick" 
class Objects_Finder:

	def __init__(self):
		rospy.init_node('Objects_Finder', anonymous=True)
		self.tfe = tf.TransformListener()
		self.camera_model = image_geometry.PinholeCameraModel()
		self.beacon_pub = rospy.Publisher(DETECTED_OBS, Marker, queue_size=10)
		self.depthimage = np.zeros((480,640,1), np.uint8)
		self.colorimage = np.zeros((480,640,3), np.uint8)
		self.marked = Set()
		self.bridge = CvBridge()
		self.depth_sub = rospy.Subscriber(DETECTED, Detections, self.get_detection)
		self.topick_sub = rospy.Subscriber(TOPICK_NODE, String, self.get_item)
		self.topick = "none"
		self.image_sub = rospy.Subscriber(RGB_IMAGE_TOPIC, Image, self.callback, queue_size=1, buff_size=480*640*8)
		self.depth_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.get_depth)
		self.cam_sub = rospy.Subscriber(CAMERA_TOPIC, CameraInfo, self.get_Cam)
		self.depth_sub = rospy.Subscriber(DETECTED, Detections, self.get_detection)
		self.puba = rospy.Publisher("Objects", Objects, queue_size=10)
		self.pub2 = rospy.Publisher("ObjectsMap", Objects, queue_size=10)
		self.flag = 0;

	def get_item(self, item):
		self.topick = item.data
		# Initialise the depth image 

	def get_Cam(self, CameraInfo):
		camera_info = CameraInfo
		self.camera_model.fromCameraInfo(camera_info)
		self.flag = 1;

	def get_Pose(self):
		t = self.tfe.getLatestCommonTime(CAMERA, MAP)
		position, quaternion = self.tfe.lookupTransform(MAP, CAMERA,  t)		
		return position, quaternion

	def get_Object_local(self,data,objectheight ,objectwidth):
		obsArray = Objects()
		ifzero = 0
		for xa in range(0, len(data.detections)):
			y = data.detections[xa].y
			h = data.detections[xa].height 
			x = data.detections[xa].x 
			w = data.detections[xa].width 
			cropped_img = self.depthimage[y-h/3:y+h/3, x-w/3:x+w/3]
			cimg = self.colorimage[y-h/2:y+h/2, x-w/2:x+w/2]

			cropped_img.setflags(write=1)
			deptharray = np.asarray(cropped_img)

			depth = np.nanmean(deptharray) 
			objectheight.append(h)
			objectwidth.append(w)

			a = FACTOR1
			b = FACTOR2
			Xa=(x-320)*a*depth
			Ya=(y-240)*b*depth

			depth = depth

			if(depth == 0 ):
				ifzero = ifzero + 1
			else:
				ObjectA = Object()
				ObjectA.name = data.detections[xa].class_name
				ObjectA.x= Xa
				ObjectA.y= Ya
				ObjectA.z = depth
				ObjectA.camera_x= x
				ObjectA.camera_y = y

				if(self.flag == 1):
					point3dH1 = self.camera_model.projectPixelTo3dRay((ObjectA.camera_x, ObjectA.camera_y+h/2))
					point3dH2 = self.camera_model.projectPixelTo3dRay((ObjectA.camera_x, ObjectA.camera_y-h/2))
					height = point3dH1[1] - point3dH2[1]
					point3dW1 = self.camera_model.projectPixelTo3dRay((ObjectA.camera_x+w/2, ObjectA.camera_y))
					point3dW2 = self.camera_model.projectPixelTo3dRay((ObjectA.camera_x-w/2, ObjectA.camera_y))
					width = point3dW1[0] - point3dW2[0]
					point3d = self.camera_model.projectPixelTo3dRay((ObjectA.camera_x, ObjectA.camera_y))

				else:
					print("none")


				obsArray.Objects.append(ObjectA)

		obsArray.length = len(data.detections) - ifzero

		return obsArray,objectheight ,objectwidth


	def debug_makers(self,obsArray,position,quaternion,xb,obsArrayMap,objectheight ,objectwidth):
		print("		")

		if(obsArray.Objects[xb].name == self.topick):

			marker = Marker()
			marker.id = xb
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
			marker.pose.position.x = position[0]+obsArray.Objects[xb].z
			marker.pose.position.y = position[1]-obsArray.Objects[xb].x
			marker.pose.position.z = position[2]-obsArray.Objects[xb].y

			if(self.flag == 1):
				#print(obsArray.Objects[xb].camera_x)
				#print(obsArray.Objects[xb].camera_y)#
				point3dH1 = self.camera_model.projectPixelTo3dRay((obsArray.Objects[xb].camera_x, obsArray.Objects[xb].camera_y+objectheight[xb]/2))
				point3dH2 = self.camera_model.projectPixelTo3dRay((obsArray.Objects[xb].camera_x, obsArray.Objects[xb].camera_y-objectheight[xb]/2))
				height = point3dH1[1] - point3dH2[1]
				#print(height)
				point3dW1 = self.camera_model.projectPixelTo3dRay((obsArray.Objects[xb].camera_x+objectwidth[xb]/2, obsArray.Objects[xb].camera_y))
				point3dW2 = self.camera_model.projectPixelTo3dRay((obsArray.Objects[xb].camera_x-objectwidth[xb]/2, obsArray.Objects[xb].camera_y))
				height = point3dW1[0] - point3dW2[0]
				#print(height)
				point3d = self.camera_model.projectPixelTo3dRay((obsArray.Objects[xb].camera_x, obsArray.Objects[xb].camera_y))
				#print(point3d)

			print("		"+(obsArray.Objects[xb].name))
			print("		X="+str(marker.pose.position.x))
			print("		Y="+str(marker.pose.position.y))
			print("		Z="+str(marker.pose.position.z))
			self.beacon_pub.publish(marker)
		return obsArray,obsArrayMap

	def get_Object_Global(self,data,obsArray,position,quaternion,objectheight ,objectwidth):
		obsArrayMap = Objects()
		for xb in range(0, len(obsArray.Objects)):
			ObjectB = Object()
			ObjectB.name = obsArray.Objects[xb].name
			obsArray,obsArrayMap=self.debug_makers(obsArray,position,quaternion,xb,obsArrayMap,objectheight ,objectwidth)

			ObjectB.x= position[0]+obsArray.Objects[xb].z
			ObjectB.y= position[1]-obsArray.Objects[xb].x
			ObjectB.z = position[2]-obsArray.Objects[xb].y


		obsArrayMap.length = obsArray.length
		return obsArray,obsArrayMap

	def get_detection(self, data):
		print("get detection")

		try:
			t = self.tfe.getLatestCommonTime(CAMERA, MAP)
			position, quaternion = self.tfe.lookupTransform(MAP, CAMERA,  t)	
			print("found")
			objectheight = []
			objectwidth = []
			obsArray, objectheight ,objectwidth  = self.get_Object_local(data,objectheight,objectwidth)
			obsArray,obsArrayMap = self.get_Object_Global(data,obsArray,position,quaternion,objectheight ,objectwidth)
			
			self.puba.publish(obsArray)
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
		try:
			self.colorimage = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)	


def main(args):
	ic = Objects_Finder()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
