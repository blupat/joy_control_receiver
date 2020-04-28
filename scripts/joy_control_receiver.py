#!/usr/bin/env python
#
# =======================================================================
#   @file   joy_control_receiver.py
#   @brief  
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rospy
from sensor_msgs.msg import Joy
from pimouse_control.srv import PiMouseCmd, PiMouseCmdRequest, PiMouseCmdResponse


class JoyControlReceiver(object):

    __slots__ = ['_joy_subscriber']

    def __init__(self):
        self._joy_subscriber = rospy.Subscriber('/joy', Joy, self.receive_joy_callback)

    def receive_joy_callback(self, message):
        # print('{0}, {1}, {2}, {3}'.format(
        #    message.axes[0], message.axes[1], message.axes[2], message.axes[3]))
        
        forward = message.axes[1] * 0.450
        rotation = 3.141592 * message.axes[2] * 90.0 / 180.0
        print('{0} [m/s], {1} [rad/s]'.format(forward, rotation))
        req = PiMouseCmdRequest()
        req.on = True
        req.run = False
        req.face = False
        req.forward = forward
        req.rotation = rotation
        rospy.ServiceProxy('/pimouse_cmd', PiMouseCmd).call(req)

    def run(self):
        rospy.spin()


if __name__=='__main__':
    rospy.init_node('joy_control_receiver')
    joy_con = JoyControlReceiver()
    joy_con.run()

