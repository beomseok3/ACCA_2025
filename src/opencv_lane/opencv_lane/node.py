#!/usr/bin/env python

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
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
   
  def get_img_cb(self, msg):
    #self.get_logger().info('---')
 
    cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    img_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    '''
    cv2.imshow("grayscale", img_gray)
    
    cv2.waitKey(1)
    '''
    img_msg = self.bridge.cv2_to_imgmsg(img_gray)
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