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
OBS = "Objects_final"
DEPTH_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
RGB_IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
MAP = "/map"
CAMERA = "/head_rgbd_sensor_rgb_frame"

class image_converter:

	def __init__(self):
		rospy.init_node('image_converter3', anonymous=True)

		self.depth_sub = rospy.Subscriber(OBS, Object, self.get_detection)

	
	def get_detection(self, data):
		print("xyz")
		print(data.x)
		print(data.y)
		print(data.z)
		print("height")
		print(data.camera_y)
		print("width")
		print(data.camera_x)




def main(args):
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
