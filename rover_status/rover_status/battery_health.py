#!/usr/bin/env python3
import rclpy    
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class health_data(Node):
    def __init__(self):
        super().__init__("check_health_status")
        self.msg = String()
        self.subscription = self.create_subscription(Float32MultiArray, "/battery_status", self.callback_,10)
        self.pub_ = self.create_publisher(String, "/battery_warning", 10)
        self.timer_ = self.create_timer(1, self.publish_data)
        # data is published every second
        self.get_logger().info("Health data node started")
        # creating a subscription to 'topic' to get array containing battery level and temperature
        self.subscription


    def callback_(self, array): # creating different messages for different battery levels
        lvl = array.data[0]
        msg = ""
        if lvl <= 5:
            msg = f"Critical battery level: {lvl}%"
        elif lvl <= 20:
            msg = f"Warning: Battery level {lvl}%"
        else:
            msg = f"Battery level healthy: {lvl}%"
        self.msg.data = msg 


    def publish_data(self):
        self.pub_.publish(self.msg)
    

def main(args=None):
    rclpy.init(args=args)
    node1 = health_data()
    rclpy.spin(node1)
    rclpy.shutdown()