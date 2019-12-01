import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
count =0
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2
class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/garbagecollector/camera1/image_raw",Image,self.callback)
  def callback(self,data):
    global count
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(cv_image.shape)
      cv2.imwrite("images/image_{}.jpg".format(count), cv_image)
      print("Saved image {}".format(count+1))

      count+=1
    except CvBridgeError as e:
      print(e)
if __name__=='__main__':
    ic = image_converter()
    rospy.init_node('image_collector')
    rospy.spin()
