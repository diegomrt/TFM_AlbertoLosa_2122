#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from ur_msgs.srv import *

def turn(bit):
    rospy.wait_for_service('ur_hardware_interface/set_io')
    try:
        set_pinza = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)
        resp1 = set_pinza(1, 0, bit)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        bit = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"%(bit))
    print("Response: %s"%(turn(bit)))