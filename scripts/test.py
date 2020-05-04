#!/usr/bin/env python

import rospy

import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import std_srvs.srv
from cv_bridge import CvBridge
from glasses.msg import DetectionBoxList, DetectionBox
import cv2
import cPickle
from threading import Thread, Lock
import random

class FeatureMatcher():

	def __init__(self):
		self._cv_bridge = CvBridge()

		self._sub = rospy.Subscriber('/camera/color/image_raw', Image, self.imageCallback, queue_size=1)
		self.image_pub = rospy.Publisher("feature_image",Image, queue_size=1)

		rospy.Service('store_scene', std_srvs.srv.Empty, self.store_current_scene_callback)
		rospy.Service('save_scenes', std_srvs.srv.Empty, self.save_current_scenes_callback)
		self.first_run = True

		self.orb = cv2.ORB_create()
		self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

		self.stored_images = []
		self.stored_kp = []
		self.stored_des = []
		self.stored_imageswkp = []

		self.mutex = Lock()


	def store_current_scene_callback(self, req):

		self.mutex.acquire()

		print self.current_image.shape

		# We only want to detect features in a small box
		center_x = self.current_image.shape[1]/2
		center_y = self.current_image.shape[0]/2
				
		img_focus = self.current_image[center_y-100:center_y+100, center_x-100:center_x+100]

		kp_focus, des_focus = self.orb.detectAndCompute(img_focus, None)

		self.stored_images.append(img_focus)
		self.stored_kp.append(kp_focus)
		self.stored_des.append(des_focus)

		modified_img = img_focus
		modified_img = cv2.drawKeypoints(img_focus, kp_focus, modified_img, color=(0,255,0), flags=0)

		self.stored_imageswkp.append(modified_img)

		self.mutex.release()
		
		print "scene stored"

		return std_srvs.srv.EmptyResponse()


	def save_current_scenes_callback(self, req):

		if not self.stored_images :
			print "no stored scenes to save"
			return std_srvs.srv.EmptyResponse()
		else:
			print "Saving image and keypoints in the scenes..."

			self.mutex.acquire()

			i = 1
			for img, kp in zip(self.stored_images, self.stored_kp):

				self.save_image_and_keypoints(img, kp, str(i))
				i=i+1

			self.mutex.release()

			print "Image and Keypoints saved."

			return std_srvs.srv.EmptyResponse()


	def find_image(self, req):

		return std_srvs.srv.EmptyResponse()		


	def save_image_and_keypoints(self, image, keypoints, name):

		index = []
		for point in keypoints:
			temp = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
			index.append(temp)

		f = open("/home/nuc/catkin_ws/detection/data/" + name +"_keypoints.txt", "w")
		f.write(cPickle.dumps(index))
		f.close()

		cv2.imwrite("/home/nuc/catkin_ws/detection/data/" + name +".png", image)


	def imageCallback(self, msg):

		img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
		self.current_image = img
        
		kp, des = self.orb.detectAndCompute(img, None)

		

		if not self.stored_images:
			new_img = img
 			new_img = cv2.drawKeypoints(img, kp, new_img, color=(0,255,0), flags=0)

 			image_out = self._cv_bridge.cv2_to_imgmsg(new_img,"bgr8")
 			image_out.header = msg.header
			self.image_pub.publish(image_out)
		else:
			# find the keypoints and compute the descriptors with ORB
			
			self.mutex.acquire()
			st_img = self.stored_images[0]
			st_kp = self.stored_kp[0]
			st_des = self.stored_des[0]
			st_imgwkp = self.stored_imageswkp[0]

 			matches = self.bf.match(st_des, des)
 			matches = sorted(matches, key = lambda x:x.distance)

 			new_img = img
 			new_img = cv2.drawKeypoints(img, kp, new_img, color=(0,255,0), flags=0)
 			img2 = new_img
 			img2 = cv2.drawMatches(st_imgwkp, st_kp, new_img, kp, matches[:10], img2, flags=2)

			image_out = self._cv_bridge.cv2_to_imgmsg(img2,"bgr8")

			image_out.header = msg.header
			self.image_pub.publish(image_out)

			self.mutex.release()

	def run(self):
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('object_detection')
	feature_matcher = FeatureMatcher()
	feature_matcher.run()