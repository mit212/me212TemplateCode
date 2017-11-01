#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for de-noising
# Luke Roberto Oct 2017

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('hough_transform', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def main():
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, rosHTransformCallback)
    print("Subscribing")
    rospy.spin()



def rosHTransformCallback(msg):
    # 1. convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # 2. visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3) 
    
    # 3. Hough Transform
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)  # Convert to grascale image
    cv2.imshow("Gray_Image", gray)
    cv2.waitKey(3) 
    
    edges = cv2.Canny(gray,10,500,apertureSize = 3)   # Canny edge detector to make it easier for hough transform to "agree" on lines
    cv2.imshow("Canny_Image", edges)
    cv2.waitKey(3) 


    lines = cv2.HoughLines(edges,1,np.pi/180,100)     # Run Hough Transform
    num_lines = 0;
    shape = lines.shape
    for i in range(shape[0]):                         # Plot lines over original feed
        for rho,theta in lines[i]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
            num_lines += 1
            
    cv2.imshow("Line_Detected_Image", cv_image)
    cv2.waitKey(5)
    print("Detecting Lines...")
    
if __name__=='__main__':
    main()
