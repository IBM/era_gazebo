#!/usr/bin/env python


# Copyright 2018 IBM

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy

import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from era_gazebo.msg import DetectionBoxList, DetectionBox
import cv2

import numpy as np
import os
import tensorflow.compat.v1 as tf
import time
import random

import object_detection
from object_detection.utils import ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

NUM_CLASSES = 90

class ObjectDetectionTF():
	
	def __init__(self):
		self._cv_bridge = CvBridge()
		
		model_zoo_path = rospy.get_param('~model_zoo_path', '~/tensorflow/models')
		model_path = rospy.get_param('~model')
		use_gpu = rospy.get_param('~use_gpu', False)
		self.detection_treshold = rospy.get_param('~detection_treshold', 0.5)
		self.use_timers = rospy.get_param('~use_timers', False)

		self.mask_enabled = True

		if use_gpu:
			gpu_devices = tf.config.experimental.list_physical_devices('GPU')
			for device in gpu_devices: tf.config.experimental.set_memory_growth(device, True)
			self.device = '/GPU:0'
			os.environ["CUDA_VISIBLE_DEVICES"] = "0" 
		else:
			self.device = '/CPU:0'
			os.environ["CUDA_VISIBLE_DEVICES"] = "-1" 

		if "kitti" in model_path:
			PATH_TO_LABELS = os.path.join(model_zoo_path, 'research/object_detection/data', 'kitti_label_map.pbtxt')
		else:
			PATH_TO_LABELS = os.path.join(model_zoo_path, 'research/object_detection/data', 'mscoco_label_map.pbtxt')

		PATH_TO_CKPT = model_path
		
		with tf.device(self.device):
			self.detection_graph = tf.Graph()
			with self.detection_graph.as_default():
				od_graph_def = tf.GraphDef()
				with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
					serialized_graph = fid.read()
					od_graph_def.ParseFromString(serialized_graph)
					tf.import_graph_def(od_graph_def, name='')
					
				label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
				categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
				self.category_index = label_map_util.create_category_index(categories)
				
				config = tf.ConfigProto()
				
				self.sess = tf.Session(config=config) 
					
			self._sub = rospy.Subscriber('image_input', Image, self.imageCallback, queue_size=1, buff_size=52428800)
			self._pub = rospy.Publisher('detected_objects', DetectionBoxList, queue_size=1)
			self.image_pub = rospy.Publisher("detection_image",Image, queue_size=1)
			
			try:
				detection_masks = self.detection_graph.get_tensor_by_name('detection_masks:0') 
			except:
				self.mask_enabled = False
		   

	def imageCallback(self, msg):
		
		with tf.device(self.device):
			cv_image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
			image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
			image_np = np.asarray(image)
			image_np_expanded = np.expand_dims(image_np, axis=0)
			
			image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
			
			if self.use_timers: 
					t1 = time.time()

			if self.mask_enabled:
				detection_masks = self.detection_graph.get_tensor_by_name('detection_masks:0') #included masks from TF graph 
				(boxes, scores, classes, num, masks) = self.sess.run(
				[detection_boxes, detection_scores, detection_classes, num_detections,detection_masks],
				feed_dict={image_tensor: image_np_expanded})
			else:
				(boxes, scores, classes, num) = self.sess.run(
				[detection_boxes, detection_scores, detection_classes, num_detections],
				feed_dict={image_tensor: image_np_expanded})
			
			if self.use_timers:
				t2 = time.time()
				print(' >>>>> Using: >>>>>>', self.device)
				print(' >>>>> Model inference time (in sec): >>>>>> ', t2-t1)

			boxes=np.squeeze(boxes)
			classes =np.squeeze(classes)
			scores = np.squeeze(scores)
					   
			if self.mask_enabled == True:
				masks = np.squeeze(masks) #squeeze masks similar to boxes
				
			result_out = DetectionBoxList()
			result_out.header = msg.header
			dim = image.shape[0:2]
			height, width = dim[0], dim[1]
			
			i = 0
			for bb, sc, cl in zip(boxes,scores,classes) :
				
				if(sc > self.detection_treshold):
					pose = DetectionBox()
					pose.left = int(bb[1]*width)
					pose.right = int(bb[3]*width)
					pose.top = int(bb[0]*height)
					pose.bottom = int(bb[2]*height)

					pose.type = self.category_index[cl]['name']
					cropped_image = image[pose.top:pose.bottom, pose.left:pose.right]
					
					if self.mask_enabled == True: #runs when only mask is enabled
						#resize the mask accd to boxW and boxH
						ma = masks[i]  #get the present mask
						boxW = pose.right - pose.left #mask width
						boxH = pose.bottom - pose.top #mask height
						#interpolate mask of 15x15 to box dimensions and convert to binary mask
						mask_img = cv2.resize(ma,(boxW,boxH),interpolation=cv2.INTER_NEAREST)
						
						#colors for the image masks
						colors=[[0, 255, 0],[0, 0, 255],[255, 0, 0],[0, 255, 255],[255, 255, 0],[255, 0, 255],[80, 70, 180],[250, 80, 190],[245, 145, 50],[70, 150, 250],[50, 190, 190]]
						#added few colors
						colors = np.array(colors) #convert to array from list
						color_selected = colors[np.random.randint(len(colors),size=1)] #randomly select one color for this object and value of the color
						#mix the image with the color and update this only if the mask is enabled
						img_seg_updated = ((0.4 * color_selected) + (0.6 * cropped_image)).astype("uint8") #combining the color with cropped image
						#writing the maksed image to original image
						for h in range(boxH): #box height
							for w in range(boxW): #box width
								if mask_img[h,w] >0.5: #if the mask is enabled then update the image with new mask generated
									image[pose.top+h,pose.left+w] = img_seg_updated[h,w] 

					result_out.detection_list.append(pose)

					cv2.rectangle(image, (pose.left, pose.top), (pose.right, pose.bottom), (255,0,0), 2, 1) #decreased the thickness from 4 to 2
					cv2.putText(image, str(pose.type), ( int(pose.left+30), int(pose.top+30) ), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(255,0,0),5)
				i = i+1    #go to the next mask
					

			if result_out.detection_list:
				self._pub.publish(result_out)
					
				image_out = self._cv_bridge.cv2_to_imgmsg(image,"rgb8")
				image_out.header = msg.header
				self.image_pub.publish(image_out)


	def run(self):
		rospy.spin()

if __name__ == '__main__':

	rospy.init_node('object_detection')
	obj_det = ObjectDetectionTF()
	obj_det.run()