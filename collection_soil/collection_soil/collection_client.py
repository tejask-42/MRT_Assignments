#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from functools import partial
from mars_srv.srv import Location
from std_msgs.msg import String

class collection(Node):
    def __init__(self):
        super().__init__("collection_client_node")
        self.client = self.create_client(Location, "target")
        # creating a client which requests Location data
        self.request = None
        self.future = None
        # creating request and future parameters to use later
        self.parse_() # start taking user input


    def parse_(self):
        n = int(input("Enter number of targets: "))
        for i in range(n):
            self.call_service() # call service for every target
        self.get_logger().info("Soil collection program completed")
            


    def call_service(self):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        self.request = Location.Request()
        self.request.target_x = float(input("Enter x coordinate of target: "))
        self.request.target_y = float(input("Enter y coordinate of target: "))
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(partial(self.callback_location))

    def callback_location(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = collection()
    rclpy.spin(node)
    rclpy.shutdown()