#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from detection.msg import DetectionBoxList, DetectionBox
from detection.srv import SceneBBox

focus_image_size = 400


class HMI():

	def __init__(self):

		self.cmd_sub = rospy.Subscriber('in_cmd', String, self.cmdCallback, queue_size=1)
		
		self.id = 0

		rospy.wait_for_service('store_scene_bbox')

	def cmdCallback(self, msg):

		center_x = 640/2
		center_y = 480/2

		bbox = DetectionBox()
		bbox.header.frame_id = ""
		bbox.header.stamp = rospy.get_rostime()
		bbox.type = "note"
		bbox.id = self.id
		self.id = self.id+1
		bbox.left = center_y-focus_image_size/2
		bbox.right = center_y+focus_image_size/2
		bbox.top = center_x-focus_image_size/2
		bbox.bottom = center_x+focus_image_size/2

		req = bbox 
		
		try:
			store_scene_bbox = rospy.ServiceProxy('store_scene_bbox', SceneBBox)
			resp = store_scene_bbox(req)
			print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e




	def run(self):
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('hmi')
	hmi = HMI()
	hmi.run()
