#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from reconbot_control.srv import PIDValues.srv
from dynamixel_driver import dynamixel_io

def set_pid_values(req):
    print "motorID = %s. Setting PID values: P = %s, I = %s and D = %s"%( req.ID, req.P, req.I, req.D, req.ID)
    motors=dynamixel_io.DynamixelIO('/dev/ttyUSB0',1000000)
    motors.set_p_gain(req.ID, req.P)
    motors.set_i_gain(req.ID, req.I)
    motors.set_d_gain(req.ID, req.D)
    """
     Sets the value of integral action of PID controller.
     Gain value is in range 0 to 254.
    """
    return True

def pid_server():
    rospy.init_node('pid_server')
    s = rospy.Service('pid_values', PIDValues, set_pid_values)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    pid_server()
