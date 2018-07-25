#!/usr/bin/env python

from __future__ import division

import rospy
from sensor_msgs.msg import LaserScan
#import math
#import json
#import sys
#import numpy as np
#from sensor_msgs.msg import PointCloud2, PointField
#from std_msgs.msg import Header
#from sensor_msgs.point_cloud2 import create_cloud_xyz32


class TransformLaser(object):
    
    def __init__(self):
        self.laser = LaserScan()
        self.laser_sub = rospy.Subscriber('/era/laser/scan', LaserScan, self.laser_callback)
        self.laser_pub = rospy.Publisher('/era/laser/transformed_scan', LaserScan, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer() # buffer for up to 10 secs
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def get_transform(self):
        try:
            self.transform = self.tf_buffer.lookup_transform('base_link',           # target frame
                                                             'laser_link',          # source frame
                                                             rospy.Time(0),         # get the latest available transform
                                                             rospy.Duration(1.0))   # wait for 1 sec
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerror('Error getting transform')
            print "ERROR!"
        
    def laser_callback(self, msg):
        self.laser = msg
        self.get_transform()
        #rospy.loginfo(msg)
        transformed_laser = LaserScan()
        transformed_laser.header = msg.header
        
        self.laser_pub.publish(msg)


rospy.init_node('transform_laser_data')
rate = rospy.Rate(1) # Hz
tl = TransformLaser()

while not rospy.is_shutdown():
    rate.sleep()
