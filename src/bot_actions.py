#!/usr/bin/env python

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
