#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

def pick():
    rospy.wait_for_service('/garbagecollector_pick')
    try:
        add_two_ints = rospy.ServiceProxy('/garbagecollector_pick', Empty)
        resp1 = add_two_ints()
        return resp1
    except rospy.ServiceException:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
   pick()