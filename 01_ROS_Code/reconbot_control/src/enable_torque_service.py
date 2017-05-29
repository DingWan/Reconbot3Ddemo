#!/usr/bin/env python

import math
import rospy
from dynamixel_driver import dynamixel_io
from reconbot_control.srv import *

def enable_torque(req):
    motors=dynamixel_io.DynamixelIO('/dev/ttyUSB0',1000000)
    state = False

    if (req.motor_state == 0) | (req.motor_state == 9) | (req.motor_state == 12):
        valueTuples = ((4,1),(5,1),(3,1),(1,1),(2,1),(6,1))
        motors.set_multi_torque_enabled(valueTuples)
        state = True

    elif req.motor_state == 11:
        valueTuples = ((4,1),(5,1),(3,0),(1,1),(2,1),(6,1))
        motors.set_multi_torque_enabled(valueTuples)
        state = True
    elif req.motor_state == 8:
        valueTuples = ((4,1),(5,1),(3,1),(1,1),(2,1),(6,0))
        motors.set_multi_torque_enabled(valueTuples)
        state = True
    elif (req.motor_state == 2) | (req.motor_state == 3) | (req.motor_state == 4) | (req.motor_state == 5):
        valueTuples = ((4,1),(5,1),(3,0),(1,1),(2,1),(6,0))
        motors.set_multi_torque_enabled(valueTuples)
        state = True
    elif (req.motor_state == 6) | (req.motor_state == 1) | (req.motor_state == 7):
        valueTuples = ((4,1),(5,1),(3,1),(1,1),(2,0),(6,1))
        motors.set_multi_torque_enabled(valueTuples)
        state = True

    elif req.motor_state == 10:
        valueTuples = ((4,1),(5,1),(3,1),(1,1),(2,1),(6,0))
        motors.set_multi_torque_enabled(valueTuples)
        state = True

    else:
        valueTuples = ((4,0),(5,0),(3,0),(1,0),(2,0),(6,0))
        motors.set_multi_torque_enabled(valueTuples)
        state = True


    return state
    print "State of the motors changed"

def enable_torque_server():
    rospy.init_node('Enable_Torque_Server')
    srv = rospy.Service('enable_torque', EnableTorque, enable_torque)
    print "Ready to drive motors"
    rospy.spin()

if __name__=="__main__":
    enable_torque_server()
