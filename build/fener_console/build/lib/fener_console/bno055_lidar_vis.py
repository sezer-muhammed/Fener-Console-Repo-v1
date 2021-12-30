#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
import time
from datetime import datetime
from cv_bridge import CvBridge

class MyNode(Node):
    def __init__(self):
        super().__init__("sensor_visualiser")
        self.get_logger().info("The Node has been started.")
        self.timer = {"BNO055":0, "Lidar":0, "Camera_Solo":0}
        

        name = datetime.now().strftime("%B %d, %Y %H:%M:%S")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_write = cv2.VideoWriter(f"/home/sezer/CONSOLE/ROS2_videos/sensor_visualiser_{name}.mp4",fourcc, 30.0, (1920, 1080))
        self.br = CvBridge()

        self.img = np.zeros((714, 1270, 3), np.uint8)
        self.bno055_data = np.zeros(6)
        self.lidar_data = np.zeros((1, 2))

        self.create_subscription(Float32MultiArray, "BNO055/all", self.bno055_callback, 1)
        self.create_subscription(Float32MultiArray, "Lidar/raw", self.lidar_callback, 1) 
        self.create_subscription(CompressedImage, "cam/solo", self.camera_callback, 1) 

        self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        frame = np.zeros((1080, 1920, 3), np.uint8)
        
        scale = 0.3
        center_x = 325
        center_y = 600
        lidar_tmp = self.lidar_data
        cv2.circle(frame, (center_x, center_y), int(500 * scale), (40, 55, 10), 2, cv2.LINE_AA)
        cv2.circle(frame, (center_x, center_y), int(1000 * scale), (40, 55, 10), 2, cv2.LINE_AA)
        cv2.circle(frame, (center_x, center_y), int(1500 * scale), (40, 55, 10), 2, cv2.LINE_AA)
        cv2.circle(frame, (center_x, center_y), int(2000 * scale), (40, 55, 10), 2, cv2.LINE_AA)
        frame[:, center_x*2:] = 0
        for point in lidar_tmp:
            lidar_point_x = int(np.sin(point[0] * np.pi / 180) * point[1] * scale) + center_x
            lidar_point_y = int(-np.cos(point[0] * np.pi / 180) * point[1] * scale) + center_y
            if lidar_point_x > 0 and lidar_point_x < 2 * center_x and lidar_point_y > 0 and lidar_point_y < 2 * center_y:
                cv2.circle(frame, (lidar_point_x, lidar_point_y), (3), (100, 40, 200), -1, cv2.LINE_AA)
        cv2.putText(frame, "Lidar data with scale {}".format(scale), (80, 1040), cv2.FONT_HERSHEY_COMPLEX, 1, (100, 60, 220), 1, cv2.LINE_AA)
        cv2.putText(frame, "Circles show 0.5 1 1.5 and 2 meters", (10, 1070), cv2.FONT_HERSHEY_COMPLEX, 1, (40*4, 55*4, 10*4), 1, cv2.LINE_AA)
        #cv2.circle(frame, (center_x, center_y), 8, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.rectangle(frame, (int(center_x - 70 * scale), int(center_y - 140 * scale)), (int(center_x + 70 * scale), int(center_y + 140 * scale)), (200, 200, 200), -1, cv2.LINE_AA)

        frame[:714, 650:650+1270, :] = self.img
        cv2.putText(frame, "Solo Camera Img", (670, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (4, 190, 255), 2, cv2.LINE_AA)
        


        bno055_heading = self.bno055_data[3] 
        heading_x = int(np.sin(bno055_heading * np.pi / 180) * 100)
        heading_y = int(np.cos(bno055_heading * np.pi / 180) * 100)
        cv2.circle(frame, (110, 110), 100, (25, 190, 180), 2, cv2.LINE_AA)
        cv2.line(frame, (110, 110), (110 + heading_x, 110 - heading_y), (25, 210, 210), 2, cv2.LINE_AA)
        cv2.putText(frame, "BNO055 Heading", (5, 240), cv2.FONT_HERSHEY_COMPLEX, 0.8, (50, 210, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "Angle: {}, Turns: {}".format(int(bno055_heading) % 365, int(bno055_heading) // 365), (5, 280), cv2.FONT_HERSHEY_COMPLEX, 0.8, (50, 210, 255), 1, cv2.LINE_AA)

        cv2.line(frame, (center_x * 2, 0), (center_x * 2, 1080), (127, 127, 127), 2, cv2.LINE_AA)

        for i, sensor in enumerate(self.timer):
            delta_time = time.time() - self.timer[sensor]
            if delta_time > 1:
                cv2.putText(frame, sensor + " is not publishing!", (665, 1065 - i * 35), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, sensor + " is publishing..", (665, 1065 - i * 35), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        self.video_write.write(frame)
        cv2.imshow("sensor_visualiser", cv2.resize(frame, (1280, 720)))
        cv2.waitKey(1)

    def camera_callback(self, msg):
        self.img = cv2.cvtColor(cv2.resize(self.br.compressed_imgmsg_to_cv2(msg), (1270, 714)), cv2.COLOR_RGB2BGR)
        self.timer["Camera_Solo"] = time.time()

    def bno055_callback(self, msg):
        self.bno055_data = msg.data
        self.timer["BNO055"] = time.time()

    def lidar_callback(self, msg):
        self.lidar_data = np.array(msg.data).reshape(-1, 2)
        self.timer["Lidar"] = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()