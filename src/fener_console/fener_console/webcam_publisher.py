# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Range
import numpy as np

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('range_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Range, 'range', 1)
      
    # We will publish a message every 0.1 seconds
    timer_period = 1/3  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         


    self.msg = Range()

    
    self.msg.header.frame_id = "Camera Left"
    self.msg.radiation_type = 0
    self.msg.field_of_view = np.deg2rad(13.0) 
    self.msg.min_range = 0.1
    self.msg.max_range = 1.0
    self.msg.range = 0.5
   
  def timer_callback(self):
    self.msg.header.stamp = self.get_clock().now().to_msg()
    self.msg.range = np.random.randint(0, 100) / 100
    print(self.msg.range)
    self.publisher_.publish(self.msg)
  
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