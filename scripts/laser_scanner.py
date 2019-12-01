#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

min_distance = 9
def min_range_index(ranges):
    print(ranges[360])
    ranges = [x for x in ranges if not math.isnan(x)]
    return min(ranges), ranges.index(min(ranges))
def laserCallback(scan_data):
    global min_distance
    min_value, index = min_range_index(scan_data.ranges)
    min_distance = min_value
    loop_rate = rospy.Rate(10)
    print(min_distance, index)
    loop_rate.sleep()
if __name__=='__main__':
    rospy.init_node('Ranging')
    sub = rospy.Subscriber('/garbagecollector/laser/scan', LaserScan, laserCallback)
    pub = rospy.Publisher('/garbagecollector/cmd_vel', Twist, queue_size=10)
    rospy.spin()
#-1.57079994678
#0.00436940183863