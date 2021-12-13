#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray #int8 for small datas

import numpy as np
from pynput.keyboard import Listener, Key

class MyNode(Node):
    def __init__(self):
        super().__init__("driver_keyboard")
        self.get_logger().info("driver_keyboard Node has been started.")

        self.key_array = ["'w'", "'a'", "'s'", "'d'", Key.space, Key.ctrl_r, Key.shift_r]
        self.press_array = [0, 0, 0, 0, 0, 0, 0]
        self.msg = Int8MultiArray()

        self.publisher = self.create_publisher(Int8MultiArray, "drive_command/keyboard", 1)
        with Listener(on_press = self.on_press, on_release = self.on_release) as listener:
            listener.join()
    
    def on_release(self, the_key):
        try:
            index = self.key_array.index(str(the_key))
        except:
            try:
                index = self.key_array.index(the_key)
            except:
                pass
        try:
            self.press_array[index] = 0
            self.msg.data = self.press_array
            self.publisher.publish(self.msg)
        except:
            pass
        

    def on_press(self, the_key):
        try:
            index = self.key_array.index(str(the_key))
        except:
            try:
                index = self.key_array.index(the_key)
            except:
                pass
        try:
            self.press_array[index] = 1
            self.msg.data = self.press_array
            self.publisher.publish(self.msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main() #main code