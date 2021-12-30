# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge

NODE_NAME = "webcam_publisher"

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__(NODE_NAME)

    timer_period = 1/60  # seconds
    
    self.publisher_ = self.create_publisher(Image, 'webcam/image', 1)
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    self.cam = cv2.VideoCapture(0)
    self.br = cv_bridge.CvBridge()

    self.get_logger().info((str(NODE_NAME) + " Node has been started and publishing data with constant rate at max " + str(1/timer_period)))
  
   
  def timer_callback(self):
    _, frame = self.cam.read()

    msg = self.br.cv2_to_imgmsg(frame, "bgr8")
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "Camera Left"
    self.publisher_.publish(msg)
    print()
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()