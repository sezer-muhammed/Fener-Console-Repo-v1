# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import cv2
import cv_bridge

NODE_NAME = "webcam_publisher"
FRAME = "Camera Left"

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

    self.timer_period = 1/60  # seconds
    self.frame_counter = 0


    self.declare_parameter("info_rate", 300)
    self.declare_parameter("D", None)
    self.declare_parameter("K", None)
    self.declare_parameter("R", None)
    self.declare_parameter("P", None)
    self.declare_parameter("width", None)
    self.declare_parameter("height", None)
    self.declare_parameter("camera_matrix", None)
    self.declare_parameter("distortion", None)
    self.declare_parameter("rectification", None)
    self.declare_parameter("projection", None)


    self.info_rate = self.get_parameter("info_rate").value
    self.p_D = self.get_parameter("D").value
    self.p_K = self.get_parameter("K").value
    self.p_R = self.get_parameter("R").value
    self.p_P = self.get_parameter("P").value
    self.p_width = self.get_parameter("width").value
    self.p_height = self.get_parameter("height").value
    self.p_camera_matrix = np.reshape(self.get_parameter("camera_matrix").value, (3,3)).tolist()
    self.p_distortion = self.get_parameter("distortion").value
    self.p_rectification = np.reshape(self.get_parameter("rectification").value, (3, 3)).tolist()
    self.p_projection = np.reshape(self.get_parameter("projection").value, (3, 4)).tolist()

    
    self.get_cam_msg()
    
    self.caminfo_publiser = self.create_publisher(CameraInfo, "webcam/camera_info", 1)
    self.img_publisher = self.create_publisher(Image, 'webcam/image_raw', 1)
    self.timer = self.create_timer(self.timer_period, self.timer_callback)
    self.timer = self.create_timer(self.timer_period * self.info_rate, self.timer_callback_info)
    self.add_on_set_parameters_callback(self.parameter_callback)
    
    self.cam = cv2.VideoCapture(0)
    self.br = cv_bridge.CvBridge()

    self.get_logger().info((str(NODE_NAME) + " Node has been started and publishing data with constant rate at max " + str(1/self.timer_period)))
  
  def timer_callback_info(self):
    self.get_logger().info(f"\n{NODE_NAME} Node is Alive\nTotal frame published {self.frame_counter}\nFrame id {FRAME}\n")

  def timer_callback(self):
    _, frame = self.cam.read()

    msg = self.br.cv2_to_imgmsg(frame, "bgr8")
    
    msg.header.stamp = self.get_clock().now().to_msg()
    self.cam_msg.header.stamp = self.get_clock().now().to_msg()

    msg.header.frame_id = FRAME
    self.frame_counter += 1

    self.img_publisher.publish(msg)
    self.caminfo_publiser.publish(self.cam_msg)

  def parameter_callback(self, params):
    for param in params:
      pass #TODO 
    return SetParametersResult(successful=True)

  def get_cam_msg(self):
    self.cam_msg = CameraInfo()
    self.cam_msg.header.frame_id = FRAME
    self.cam_msg.height = self.p_height
    self.cam_msg.width = self.p_width
    self.cam_msg.distortion_model = "plumb_bob"
    self.cam_msg.d = self.p_D
    self.cam_msg.k = self.p_K
    self.cam_msg.r = self.p_R
    self.cam_msg.p = self.p_P
  
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