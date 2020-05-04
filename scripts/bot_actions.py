#!/usr/bin/env python
<<<<<<< HEAD
=======
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
>>>>>>> 99c8973d3516bf669c220e791b7b95e8c2ebbeb8

from geometry_msgs.msg import Twist


DEF_SPEED        = 1.0
BACKWARD_SCALING = 0.1
TURN_SCALING     = 2.0


class BotAction(Twist):
    def __init__(self):
        super(BotAction, self).__init__()
        self.linear.x  = 0.0
        self.linear.y  = 0.0
        self.linear.z  = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0

class TnLeft(BotAction):
    def __init__(self):
        super(TnLeft, self).__init__()
        self.angular.z = DEF_SPEED * TURN_SCALING

class TnRight(BotAction):
    def __init__(self):
        super(TnRight, self).__init__()
        self.angular.z = -DEF_SPEED * TURN_SCALING

class MvForward(BotAction):
    def __init__(self):
        super(MvForward, self).__init__()
        self.linear.x = DEF_SPEED

class MvBackward(BotAction):
    def __init__(self):
        super(MvBackward, self).__init__()
        self.linear.x = -DEF_SPEED * BACKWARD_SCALING


bot_actions = {
    'TnLeft'     : TnLeft(),
    'TnRight'    : TnRight(),
    'MvForward'  : MvForward(),
    'MvBackward' : MvBackward()
}
