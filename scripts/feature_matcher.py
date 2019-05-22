#!/usr/bin/env python

import rospy

import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import std_srvs.srv
from cv_bridge import CvBridge
from detection.msg import DetectionBoxList, DetectionBox
from detection.srv import SceneBBox
import cv2
import cPickle
from threading import Thread, Lock
import random
import numpy as np
import os

focus_image_size = 400

class Scene():

	def __init__(self, image):

		print "test"



class FeatureMatcher():

	def __init__(self):
		self._cv_bridge = CvBridge()

		self._sub = rospy.Subscriber('/camera/color/image_raw', Image, self.imageCallback, queue_size=1)
		self.image_pub = rospy.Publisher("feature_image",Image, queue_size=1)
		self.stored_scenes_pub = rospy.Publisher("stored_scenes",Image, queue_size=1, latch=True)

		rospy.Service('store_scene', std_srvs.srv.Empty, self.store_current_scene_callback)
		rospy.Service('store_scene_bbox', SceneBBox, self.store_scene_bbox_callback)
		rospy.Service('save_scenes', std_srvs.srv.Empty, self.save_current_scenes_callback)
		rospy.Service('load_scenes', std_srvs.srv.Empty, self.load_scenes_callback)
		self.first_run = True

		#self.orb = cv2.ORB_create()
		self.sift = cv2.xfeatures2d.SIFT_create()
		self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
		flann_params = dict(algorithm = 1, trees = 5)      # flann enums are missing, FLANN_INDEX_KDTREE=1
		self.matcher = cv2.FlannBasedMatcher(flann_params, {})
		self.bow_train   = cv2.BOWKMeansTrainer(50) # toy world, you want more.
		self.bow_extract = cv2.BOWImgDescriptorExtractor(self.sift, self.matcher )
		self.svm = cv2.ml.SVM_create()

		self.stored_images = []
		self.stored_kp = []
		self.stored_des = []
		self.stored_imageswkp = []

		self.mutex = Lock()

	def store_scene_bbox_callback(self, req):

		self.mutex.acquire()

		print "Storing Current Scene"

		#return SceneBBox.EmptyResponse()

		img_focus = self.current_image[req.bbox.left:req.bbox.right, req.bbox.top:req.bbox.bottom]

		des_focus = []
		kp_focus, des_focus = self.sift.detectAndCompute(img_focus, None)

		if  des_focus == [] or des_focus.shape[0] < 50:
			self.mutex.release()
			return

		self.stored_images.append(img_focus)
		self.stored_kp.append(kp_focus)
		self.stored_des.append(des_focus)

		modified_img = img_focus
		modified_img = cv2.drawKeypoints(img_focus, kp_focus, modified_img, color=(0,255,0), flags=0)

		self.stored_imageswkp.append(modified_img)

		mosaic_img = self.image_mosaic(self.stored_imageswkp)

		self.trainBOW()

		self.mutex.release()
		
		image_out = self._cv_bridge.cv2_to_imgmsg(mosaic_img,"bgr8")
 		#image_out.header = msg.header
		self.stored_scenes_pub.publish(image_out)

		print "scene stored"

	def store_current_scene_callback(self, req):

		req = SceneBBox()

		bbox = DetectionBox()
		bbox.header.frame_id = ""
		bbox.header.stamp = rospy.get_rostime()
		bbox.type = "note"
		bbox.id = 0

		center_x = self.current_image.shape[1]/2
		center_y = self.current_image.shape[0]/2

		bbox.left = center_y-focus_image_size/2
		bbox.right = center_y+focus_image_size/2
		bbox.top = center_x-focus_image_size/2
		bbox.bottom = center_x+focus_image_size/2
		req.bbox = bbox

		self.store_scene_bbox_callback(req)

		return std_srvs.srv.EmptyResponse()

	def image_mosaic(self, images):

		no_images = len(images)
		width = images[0].shape[1]
		height = images[0].shape[0]
		blank_image = np.zeros((height,width,3), np.uint8)

		if no_images < 5:
			prev_img = images[0]
			for i in range(1, no_images) :
				img  = images[i]
				prev_img = np.concatenate((prev_img, img), axis=1)
			return prev_img

		image_rows = []
		for i in range(0, no_images,4) :
			prev_img = images[i]
			for j in range(1, 4) :
				if i+j >= no_images:
					img = blank_image
				else:
					img  = images[i+j]
				prev_img = np.concatenate((prev_img, img), axis=1)
			image_rows.append(prev_img)


		if len(image_rows) == 1:
			return image_rows[0]

		prev_row = image_rows[0]

		for i in range(1, len(image_rows)):
			prev_row = np.concatenate((prev_row, image_rows[i]), axis=0)

		return prev_row

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

	def load_scenes_callback(self,req):

		self.mutex.acquire()

		self.load_image_and_keypoints()

		mosaic_img = self.image_mosaic(self.stored_imageswkp)

		self.trainBOW()

		self.mutex.release()
		
		image_out = self._cv_bridge.cv2_to_imgmsg(mosaic_img,"bgr8")
 		#image_out.header = msg.header
		self.stored_scenes_pub.publish(image_out)

		print "Image and Keypoints loaded."

		return std_srvs.srv.EmptyResponse()

	def save_image_and_keypoints(self, image, keypoints, name):

		index = []
		for point in keypoints:
			temp = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
			index.append(temp)

		f = open("/home/nuc/catkin_ws/src/detection/data/" + name +"_keypoints.txt", "w")
		f.write(cPickle.dumps(index))
		f.close()

		cv2.imwrite("/home/nuc/catkin_ws/src/detection/data/" + name +".png", image)

	def load_image_and_keypoints(self):
		
		for filename in os.listdir("/home/nuc/catkin_ws/src/detection/data/"):

			filename = "/home/nuc/catkin_ws/src/detection/data/" + filename
			if filename.endswith(".png"):

				filename = os.path.splitext(filename)[0]

				img = cv2.imread(filename + ".png")
				self.stored_images.append(img)

				index = cPickle.loads(open(filename + "_keypoints.txt").read())
				kp = []

				for point in index:
					temp = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5]) 
					kp.append(temp)

   				self.stored_kp.append(kp)

   				kp, des = self.sift.compute(img, kp)

   				#if not des:
   				#	print "empty des"

   				self.stored_des.append(des)

   				new_img = img
 				self.stored_imageswkp.append(cv2.drawKeypoints(img, kp, new_img, color=(0,255,0), flags=0))

 	def add_to_training(self, image, label):

 		print "test adding image and label"


 	def trainBOW(self):
 		
		#for img, kp in zip(self.stored_images, self.stored_kp):
			#kp1, des = self.sift.compute(img, kp)
		for des in self.stored_des:
			self.bow_train.add(des)

		voc = self.bow_train.cluster()
		self.bow_extract.setVocabulary( voc )
		#print "bow vocab", np.shape(voc), voc

		traindata, trainlabels = [],[]

		i=0
		for img, kp in zip(self.stored_images, self.stored_kp):
			traindata.extend(self.bow_extract.compute(img, kp))
			trainlabels.append(i)
			i = i+1
		
		#print "svm items", len(traindata), len(traindata[0])

		self.svm.train(np.array(traindata), cv2.ml.ROW_SAMPLE, np.array(trainlabels))


	def find_image(self, img):

		if not self.stored_des:
			print "No scenese stored"
			return img

		kp = self.sift.detect(img)
		
		des = self.bow_extract.compute(img, kp)

		p = self.svm.predict(des)
		print p[1][0][0] 
		print p


		return img

		matches = self.matcher.knnMatch(self.stored_des[0], des, k=2)

		good = []
		for m_n in matches:
			if len(m_n) != 2:
				continue
			(m,n) = m_n
			if m.distance < 0.8*n.distance:
				good.append(m)


		if len(good)>MIN_MATCH_COUNT:
			
				
			src_pts = np.float32([ self.stored_kp[0][m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
			matchesMask = mask.ravel().tolist()

			h = img.shape[0]
			w = img.shape[1]
			pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			dst = cv2.perspectiveTransform(pts,M)

			img2 =  cv2.polylines(img, [np.int32(dst)],True,255,3, cv2.LINE_AA)

			draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

			img2 = cv2.drawMatches(self.stored_images[0], self.stored_kp[0], img2, kp,  good, None,**draw_params)
			
			

		else:
		    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
		    matchesMask = None
		    img2 = img
		    

		return img2

	def imageCallback(self, msg):

		img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
		self.current_image = img
        
		self.mutex.acquire()

		#if self.stored_images:
		img2 = self.find_image(img)

		center_x = img2.shape[1]/2
		center_y = img2.shape[0]/2
		cv2.rectangle(img2, (center_x-focus_image_size/2, center_y-focus_image_size/2), (center_x+focus_image_size/2, center_y+focus_image_size/2), (255,0,0), 4, 1)

		image_out = self._cv_bridge.cv2_to_imgmsg(img2,"bgr8")
		self.image_pub.publish(image_out)
		#else:	
		#	img2 = img

		self.mutex.release()
		# kp, des = self.orb.detectAndCompute(img, None)

		# if not self.stored_images:
		# 	new_img = img
 	# 		new_img = cv2.drawKeypoints(img, kp, new_img, color=(0,255,0), flags=0)

 	# 		image_out = self._cv_bridge.cv2_to_imgmsg(new_img,"bgr8")
 	# 		image_out.header = msg.header
		# 	self.image_pub.publish(image_out)
		# else:
		# 	# find the keypoints and compute the descriptors with ORB
			
		# 	self.mutex.acquire()
		# 	st_img = self.stored_images[0]
		# 	st_kp = self.stored_kp[0]
		# 	st_des = self.stored_des[0]
		# 	st_imgwkp = self.stored_imageswkp[0]

 	# 		matches = self.bf.match(st_des, des)
 	# 		matches = sorted(matches, key = lambda x:x.distance)

 	# 		new_img = img
 	# 		new_img = cv2.drawKeypoints(img, kp, new_img, color=(0,255,0), flags=0)
 	# 		img2 = new_img
 	# 		img2 = cv2.drawMatches(st_imgwkp, st_kp, new_img, kp, matches[:10], img2, flags=2)

		#image_out = self._cv_bridge.cv2_to_imgmsg(img2,"bgr8")

		#image_out.header = msg.header
		#self.image_pub.publish(image_out)

		# 	self.mutex.release()

	def run(self):
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('object_detection')
	feature_matcher = FeatureMatcher()
	feature_matcher.run()