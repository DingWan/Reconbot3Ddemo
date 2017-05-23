#!/usr/bin/env python

import math
import rospy
from dynamixel_driver import dynamixel_io
from reconbot_control.srv import EnableTorque

def enable_torque(req):
    motors=dynamixel_io.DynamixelIO('/dev/ttyUSB0',1000000)
    state = False

    if req.motor_state == 1:
        motors.set_multi_torque_enabled((4,1),(5,1),(3,1),(1,1),(2,1),(6,1))
        state = True
    elif req.motor_state == 2:
        motors.set_multi_torque_enabled((4,0),(5,1),(3,0),(1,1),(2,1),(6,1))
        state = True
    elif req.motor_state == 3:
        motors.set_multi_torque_enabled((4,1),(5,1),(3,1),(1,0),(2,1),(6,0))
        state = True
    elif req.motor_state == 4:
        motors.set_multi_torque_enabled((4,1),(5,1),(3,0),(1,1),(2,1),(6,0))
        state = True
    elif req.motor_state == 5:
        motors.set_multi_torque_enabled((4,0),(5,1),(3,0),(1,1),(2,0),(6,1))
        state = True
    else:
        motors.set_multi_torque_enabled((4,0),(5,1),(3,0),(1,0),(2,0),(6,1))
        state = True

    return state
    print "State of the motors changed"

def enable_torque_server():
    rospy.init_node('Enable_Torque_Server')
    srv = rospy.Service('enable_torque', EnableTorque, enable_torque)
    rospy.spin()

if __name__=="__main__":
    enable_torque_server()
