#!/usr/bin/env python
#
# Copyright 2018 IBM
#
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
#

import sys
import rospy
from geometry_msgs.msg import Twist
from os.path import basename

import bot_actions as ba


def rotate(node_name, namespace="r0"):
    # Starts a new node
    rospy.init_node(node_name, anonymous=True)

    topic_name = '/%s/cmd_vel_mux/input/teleop' % (namespace,)
    vel_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel_msg = ba.bot_actions['TnLeft']
        vel_publisher.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage: <script_name> <namespace>"
    else:
        try:
            #Testing our function
            rotate(basename(sys.argv[0]), sys.argv[1])
        except rospy.ROSInterruptException:
            pass
