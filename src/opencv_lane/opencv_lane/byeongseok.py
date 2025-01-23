#!/usr/bin/env python

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
 
class ImageConvertor(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.get_img_cb, 
      10)
    self.img_pub = self.create_publisher(Image, 'image_gray', 10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
   

  def regionOfInterest(self,img,vertices):

    mask = np.zeros_like(img)

    if len(img.shape) > 2: 
      channelCount = img.shape[2] #3,4해도 됨 이미지에 따라
      ignoreMaskColor = (255,) * channelCount

    else:
      ignoreMaskColor = 255
    
    cv2.fillPoly(mask, vertices, ignoreMaskColor)

    maskedImage = cv2.bitwise_and(img, mask)
    return maskedImage
  

  def grayScale(self, img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  

  def drawLine(self, img, lines):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
              cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    else:
        print("No lines detected.")




  def hough_lines(self,img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                           min_line_len,
                           max_line_gap)
    
    line_img = np.zeros((img.shape[0], img.shape[1],3), dtype = np.uint8)
    self.drawLine(line_img,lines)
    return line_img


  def weighted_img(self, img, initial_img, a = 0.8, b = 1., c = 0.):
    return cv2.addWeighted(initial_img, a, img, b, c)
  
  def get_img_cb(self, msg):
 
    cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    ### ROI Setting ####
    imshape = cv_img.shape

    # vertices = np.array([[(100,imshape[0]),
    #                       (450,320),
    #                       (550,320),
    #                       (imshape[1]-20, imshape[0])]], dtype=np.int32)

    vertices = np.array([[(100,imshape[0]),
                          (70,250),
                          (520,250),
                          (imshape[1]-20, imshape[0])]], dtype=np.int32)
    
    
    #### GRAYSCALE #####
    img_gray = self.grayScale(cv_img)
    
    #### Gaussian Blur ####
    kernelSize = 5
    blurGray = cv2.GaussianBlur(img_gray,(kernelSize,kernelSize), 0)

    #### Canny ####
    lowThreshold = 50
    highThreshold = 200
    edges = cv2.Canny(blurGray,lowThreshold,highThreshold)

    #### apply ROI ####
    mask = self.regionOfInterest(edges,vertices)
    
    
    #### Draw lines ####
    rho = 2
    theta = np.pi/180
    threshold = 90
    min_line_len = 120
    max_line_gap = 150

    lines = self.hough_lines(mask, rho, theta, threshold,
                             min_line_len, max_line_gap)


    #### Weighted over original usb_cam img ####
    lines_edges = self.weighted_img(lines, cv_img, a= 0.8, b = 1., c = 0.)

    #### Publish ####
    img_msg = self.bridge.cv2_to_imgmsg(lines_edges)
    self.img_pub.publish(img_msg)


  


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = ImageConvertor()
  
  # Spin the node so the callback function is called.
  rclpy.spin(node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()