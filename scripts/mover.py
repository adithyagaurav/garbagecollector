#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math
import time
import rosservice
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
os.environ["KERAS_BACKEND"] = "tensorflow"
from keras.models import load_model
import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.remove('/home/simulations/public_sim_ws/devel/lib/python2.7/dist-packages')
import cv2



velocity_message = Twist()
min_distance = 9
mid_value =float("inf")
garbage_segregation_model = load_model('model/garbage_classifier.h5')
garbage_classes = ['banana', 'can', 'cup', 'melon', 'tennis_ball']
bridge = CvBridge()
cv_image=[]

def garbage_segregation(image):
    global garbage_segregation_model, garbage_classes
    img_batch = []
    image = cv2.resize(image, (500,500))
    img_batch.append(image)
    img_batch = np.array(img_batch)
    pred = garbage_segregation_model.predict(img_batch)
    return garbage_classes[np.argmax(pred)]
def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return min(ranges), ranges[360]

def laserCallback(scan_data):
    global min_distance, mid_value
    min_value, mid_value = min_range_index(scan_data.ranges)
    min_distance = min_value
def imageCallback(image_data):
    global cv_image, bridge
    cv_image = bridge.imgmsg_to_cv2(image_data)

def pick():
    rospy.wait_for_service('/garbagecollector_pick')
    try:
        pick_action = rospy.ServiceProxy('/garbagecollector_pick', Empty)
        resp1 = pick_action()
        return resp1
    except rospy.ServiceException:
        print("Service call failed: %s"%e)

def rotate():
    print("robot rotates")
    loop_rate = rospy.Rate(10)
    while 1:
        print(mid_value)
        if math.isinf(mid_value):
            velocity_message.linear.x=0
            velocity_message.angular.z=0.7
            pub.publish(velocity_message)
        else:
            if mid_value < 0.28:
                velocity_message.linear.x=0
                velocity_message.angular.z=0
                pub.publish(velocity_message)
                garbage_class = garbage_segregation(cv_image)
                print("Picking up {}".format(garbage_class))
                pick()
            else:
                velocity_message.linear.x=0.2
                velocity_message.angular.z=0
                pub.publish(velocity_message)
        loop_rate.sleep()
if __name__=='__main__':
    rospy.init_node('Rotating')
    sub = rospy.Subscriber('/garbagecollector/laser/scan', LaserScan, laserCallback)
    pub = rospy.Publisher('/garbagecollector/cmd_vel', Twist, queue_size=10)
    image_sub = rospy.Subscriber("/garbagecollector/camera1/image_raw",Image,imageCallback)
    rotate()
    rospy.spin()