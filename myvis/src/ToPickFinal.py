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

class To_Pick:

	def __init__(self,topickOB):
		rospy.init_node('To_Pick', anonymous=True)
		self.puba = rospy.Publisher("topick", String, queue_size=10)
		self.topick =topickOB
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			print(self.topick)
			self.puba.publish(self.topick)
			print(self.topick)

			rate.sleep()
			print(self.topick)


	def puba(self):
		print(self.topick)
		self.puba.publish(self.topick)


def main(args):
	topickOB =(sys.argv[1])
	ic = To_Pick(topickOB)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
