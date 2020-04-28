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

    __slots__ = [
        '_joy_subscriber', '_is_on', '_is_run', '_is_face',
        '_sw_on_off', '_sw_run', '_sw_face']

    def __init__(self):
        self._joy_subscriber = rospy.Subscriber('/joy', Joy, self.receive_joy_callback)
        self._is_on = False
        self._is_run = False
        self._is_face = False
        self._sw_on_off = 0
        self._sw_run = 0
        self._sw_face = 0

    def receive_joy_callback(self, message):
        # print('{0}, {1}, {2}, {3}'.format(
        #    message.axes[0], message.axes[1], message.axes[2], message.axes[3]))
        
        if (self._sw_on_off == 1) and (message.buttons[0] == 0):
            if self._is_on:
                self._is_on = False
                self._is_run = False
                self._is_face = False
            else:
                self._is_on = True
                self._is_run = False
                self._is_face = False
        elif (self._sw_run == 1) and (message.buttons[1] == 0):
            self._is_on = True
            self._is_run = True
            self._is_face = False
        elif (self._sw_face == 1) and (message.buttons[2] == 0):
            self._is_on = True
            self._is_run = False
            self._is_face = True
        self._sw_on_off = message.buttons[0]
        self._sw_run = message.buttons[1]
        self._sw_face = message.buttons[2]

        req = PiMouseCmdRequest()
        if self._is_on and (not self._is_run) and (not self._is_face):
            forward = message.axes[1] * 0.450
            rotation = 3.141592 * message.axes[2] * 90.0 / 180.0
            print('{0} [m/s], {1} [rad/s]'.format(forward, rotation))
            req.on = True
            req.run = False
            req.face = False
            req.forward = forward
            req.rotation = rotation
        elif self._is_on and self._is_run:
            print('Run')
            req.on = True
            req.run = True
            req.face = False
            req.forward = 0.0
            req.rotation = 0.0
        elif self._is_on and self._is_face:
            print('Face')
            req.on = True
            req.run = False
            req.face = True
            req.forward = 0.0
            req.rotation = 0.0
        else:
            print('Off')
            req.on = False
            req.run = False
            req.face = False
            req.forward = 0.0
            req.rotation = 0.0
        rospy.ServiceProxy('/pimouse_cmd', PiMouseCmd).call(req)


    def run(self):
        rospy.spin()


if __name__=='__main__':
    rospy.init_node('joy_control_receiver')
    joy_con = JoyControlReceiver()
    joy_con.run()

