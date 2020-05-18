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
#import tensorflow as tf
import time
import random


from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile

#sys.path.append('/home/nuc/local/ext/tensorflow/models/research/object_detection')

#import object_detection
#from object_detection.utils import ops
#from object_detection.utils import label_map_util
#from object_detection.utils import visualization_utils as vis_util

# What model to download.
#MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'
#MODEL_NAME = 'faster_rcnn_resnet101_coco_11_06_2017'
#MODEL_NAME = 'ssd_inception_v2_coco_2018_01_28'
#MODEL_NAME = 'ssd_resnet50_v1_2018_07_03'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
#PATH_TO_CKPT = '~/catkin_ws/src/era_gazebo/models/' + MODEL_NAME + '/frozen_inference_graph.pb'

#device = '/cpu:0'      #'/cpu:0' #at present 1 device is assumed, Set to cpu if you are using cpu else set to gpu 
#os.environ["CUDA_VISIBLE_DEVICES"]="-1" # -1 set no GPU visible

mask_enabled = False #set to Flase for non-mask object detection and True for mask RCNN object detection
gpu_devices = tf.config.experimental.list_physical_devices('GPU')
for device in gpu_devices: tf.config.experimental.set_memory_growth(device, True)

NUM_CLASSES = 90

class ImageMatcher():

    def __init__(self):

        print("init")
        self.histograms = dict()
        
    def add_model(self, hist, category):
        
        #hist = cv2.calcHist([image], [0,1,2], None, [128, 128, 128], [0, 256, 0, 256, 0, 256])
        #hist = cv2.normalize(hist, hist).flatten()
        
            
        if( category not in self.histograms ):
            self.histograms[category] = []
            
        ID = random.randint(1, 100)
        self.histograms[category].append( (ID,hist) )
        return ID
                
       
    def match_and_add_model(self, image, category):

        #hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        #hist = cv2.calcHist([image], [0,1,2], None, [128, 128, 128], [0, 256, 0, 256, 0, 256])
        #hist = cv2.calcHist([image], [0], None, [64], [0, 256])
        #hist = cv2.normalize(hist, hist).flatten()

        #print hist.shape
        
        s = self.gaussian_weights(image).reshape(-1)
        colors = cv2.split(image)
        
        hist0, bins = np.histogram(colors[0].reshape(-1), 64, (0,256), weights = s)
        hist0 = hist0.astype(np.float32)
        hist0 = cv2.normalize(hist0, hist0).flatten()

        hist1, bins = np.histogram(colors[1].reshape(-1), 64, (0,256), weights = s)
        hist1 = hist1.astype(np.float32)
        hist1 = cv2.normalize(hist1, hist1).flatten()

        hist2, bins = np.histogram(colors[2].reshape(-1), 64, (0,256), weights = s)
        hist2 = hist2.astype(np.float32)
        hist2 = cv2.normalize(hist2, hist2).flatten()
        

        hist  = (hist0+ hist1 + hist2 ) /2.0

        
        if( category not in self.histograms ):
            print("Category %s does not exist in the model database" %category)
            return self.add_model(hist, category)
            
        max_comparison = 0
        
        for model in  self.histograms[category]: 
            #print "testing category %s ID %d" % (category, model[0]) 
            comparison = cv2.compareHist(hist, model[1], cv2.HISTCMP_CORREL)
            
            #print "comparison result %f " % comparison
            if comparison > max_comparison :
                max_comparison = comparison
                ID = model[0]

        #print "comparison finished with %f score" % max_comparison
        if max_comparison > 0.5:
            #print "Found %s with ID %d" % (category, ID)
            return ID
        else:
            #print ("Cannot find matching  %s max score is %f" % category, max_comparison)
            ID = self.add_model(hist, category)
            print ("Assigning new ID %d" % ID)
            return ID

    
    def gaussian_weights(self, image):
        dim = image.shape[0:2]
        x, y  = np.meshgrid(np.linspace(-1,1,dim[1]), np.linspace(-1,1,dim[0]))
        d = np.sqrt(x*x+y*y)
        sigma, mu = 0.2, 0.0
        s = np.exp(-( (d-mu)**2 / ( 2.0 * sigma**2 ) ) ) * 255
        return s


class ObjectDetectionTF():
    
    def __init__(self):
        self._cv_bridge = CvBridge()



        model_zoo_path = rospy.get_param('~model_zoo_path', '~/tensorflow/models')
        model_path = rospy.get_param('~model')
        use_gpu = rospy.get_param('~use_gpu', False)
        self.use_timers = rospy.get_param('~use_timers', False)

        if use_gpu:
            self.device = '/GPU:0'
            os.environ["CUDA_VISIBLE_DEVICES"] = "0" 
        else:
            self.device = '/CPU:0'
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1" 


        PATH_TO_LABELS = os.path.join(model_zoo_path, 'research/object_detection/data', 'mscoco_label_map.pbtxt')
        PATH_TO_CKPT = model_path

        print("PATH to ckpt: " + PATH_TO_CKPT)
        print("PATH TO LABELS " + PATH_TO_LABELS)

        #self.orb = cv2.xfeatures2d.SIFT_create()
        
        self.mask_enabled = mask_enabled
        self.matcher = ImageMatcher()
        
        with tf.device(self.device):
            self.detection_graph = tf.Graph()
            with self.detection_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')
                    
                label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
                #categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
                #self.category_index = label_map_util.create_category_index(categories)
                
                self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)



                config = tf.ConfigProto() #log_device_placement=True)
                
                if use_gpu: #only for GPU this is valid
                    config.gpu_options.allow_growth=True  #this will allocate the memory during the run time
                    #to limit GPU memory of the card choose next option "per_process_gpu_memory_fraction"
                    #config.gpu_options.per_process_gpu_memory_fraction = 0.6
                self.sess = tf.Session(config=config) 
                    
            self._sub = rospy.Subscriber('image_input', Image, self.imageCallback, queue_size=1, buff_size=52428800)
            self._pub = rospy.Publisher('detected_objects', DetectionBoxList, queue_size=1)
            self.image_pub = rospy.Publisher("detection_image",Image, queue_size=1)
            
            try:
                detection_masks = self.detection_graph.get_tensor_by_name('detection_masks:0') 
            except:
                self.mask_enabled = False
           

    def imageCallback(self, msg):
        #added imports for callback function
        #import object_detection
        #from object_detection.utils import ops
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
                
                if(sc > .30):
                    pose = DetectionBox()
                    pose.left = int(bb[1]*width)
                    pose.right = int(bb[3]*width)
                    pose.top = int(bb[0]*height)
                    pose.bottom = int(bb[2]*height)

                    pose.type = self.category_index[cl]['name']
                    cropped_image = image[pose.top:pose.bottom, pose.left:pose.right]
                    pose.id = self.matcher.match_and_add_model(cropped_image, pose.type)
                    
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
                    cv2.putText(image, str(pose.id), ( int(pose.left+30), int(pose.top+30) ), cv2.FONT_HERSHEY_SIMPLEX, 2.0,(255,0,0),5)
                i = i+1    #go to the next mask
                    #kp1, des1 = self.orb.detectAndCompute(crop_image, None)
                    #cv2.drawKeypoints(crop_image,kp1, crop_image, color=(0,255,0), flags=0)

            if result_out.detection_list:
                self._pub.publish(result_out)
                    
                # Visualization of the results of a detection.
                # vis_util.visualize_boxes_and_labels_on_image_array(
                #     image,
                #     np.squeeze(boxes),
                #     np.squeeze(classes).astype(np.int32),
                #     np.squeeze(scores),
                #     self.category_index,
                #     use_normalized_coordinates=True,
                #     min_score_thresh=.8,
                #     line_thickness=8)
            
                #img=cv2.cvtColor(image_np, cv2.COLOR_BRGB
                image_out = self._cv_bridge.cv2_to_imgmsg(image,"rgb8")
                image_out.header = msg.header
                self.image_pub.publish(image_out)

            #rospy.sleep(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('object_detection')
    obj_det = ObjectDetectionTF()
   
obj_det.run()
