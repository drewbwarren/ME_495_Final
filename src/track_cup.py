#!/usr/bin/env python
from __future__ import print_function
#import roslib
#roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String, Int64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import (
    Image, Range
)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
QSIZE=100



class image_converter:


  def __init__(self):
    self.position=Point()
    self.position.x=0
    self.position.y=0
    self.position.z=0
    self.image_pub = rospy.Publisher("cup_center",Point, queue_size=QSIZE)

    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.callback)



  def depth(self,data):
    if data.range <= 0.3:
        self.position.z = data.range
    else:
        self.position.z = 0

  def callback(self,data):

    a = 0
    b = 0
    try:

      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      hsv_image =cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      sensitivity =30
      lower_blue=np.array([0,50, 50])
      higher_blue=np.array([10, 255, 255])
      mask=cv2.inRange(hsv_image, lower_blue, higher_blue)
      mask = cv2.erode(mask, None, iterations=2)
      mask = cv2.dilate(mask, None, iterations=2)
      res = cv2.bitwise_and(cv_image,cv_image, mask= mask)


      cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
      center =None
      if len(cnts) >0:
        c=max(cnts, key =cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M=cv2.moments(c)
        center = (int(M["m10"]/M["m00"]), int(M["m01"] / M["m00"]))
        if radius >10:
          cv2.circle(cv_image, (int(x), int(y)), int(radius), (255,0,0), 2)
          cv2.circle(cv_image, center, 5, (0, 255, 0), -1)
          self.position.x = int(center[0])
          self.position.y = int(center[1])

    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
#    if cols > 60 and rows > 60 :
#      cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.imshow("ROI show", res)

    rospy.Subscriber("/robot/range/left_hand_range/state", Range, self.depth)

    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.position)
      #print(a, b)

    except CvBridgeError as e:
      print(e)

def main(args):
  image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
