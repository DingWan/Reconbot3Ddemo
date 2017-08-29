#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from reconbot_control.srv import PIDValues.srv

def set_pid_values_client(ID, P, I, D):
    rospy.wait_for_service('pid_values')
    try:
        pid_values = rospy.ServiceProxy('pid_values', PIDValues)
        resp1 = pid_values(ID, P, I, D)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [ID P I D]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        ID = int(sys.argv[1])
        P = int(sys.argv[2])
        I = int(sys.argv[3])
        D = int(sys.argv[4])
    else:
        print usage()
        sys.exit(1)
    print "motorID = %s. Setting PID values: P = %s, I = %s and D = %s"%(ID, P, I, D)
    print "Response %s"%(set_pid_values_client(ID, P, I, D))
