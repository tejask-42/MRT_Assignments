#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class battery_sub_node(Node):
    def __init__(self):
        super().__init__("battery_level_subscriber")
        self.sub_1 = self.create_subscription(Float32MultiArray, "/battery_status", self.callback_1,10)
        self.sub_2 = self.create_subscription(String, "/battery_warning", self.callback_2, 10)
        self.get_logger().info("Battery subscriber node has started")
        # battery_sub_node subscribes to 'topic' as well as 'topic2'
        # it receives battery level and temp from topic as published by battery health node
        # it receives warning data from battery health node
        self.sub_1
    def callback_1(self, array):
        self.get_logger().info(f"The battery level is {int(array.data[0])}%")
        self.get_logger().info(f"The battery temperature is {array.data[1]}°C")
        # logging info which is received

    def callback_2(self, string):
        self.get_logger().info(str(string))
        # logging the message given by battery health node

def main(args=None):
    rclpy.init(args=args)
    node = battery_sub_node()
    rclpy.spin(node)
    rclpy.shutdown()    