#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from random import uniform, randrange
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class battery_temp_data(Node):
    def __init__(self):
        super().__init__("show_battery_level")
        self.pub_ = self.create_publisher(Float32MultiArray, "/battery_status", 10)
        self.timer_ = self.create_timer(1, self.publish_data)
        self.get_logger().info("Battery, temperature node started")
        # publishes data every second to 'topic' containing random data about battery level and temperature

    def publish_data(self):
        msg = Float32MultiArray()
        battery_level = float(randrange(0, 101)) # battery level integer
        temp = uniform(-20, 80) # temp takes a random float value between -20 and 80
        msg.data = [battery_level, temp]
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = battery_temp_data()
    rclpy.spin(node)
    rclpy.shutdown()

