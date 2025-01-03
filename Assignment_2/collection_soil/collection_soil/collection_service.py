#!/usr/bin/env python3
from mars_srv.srv import Location
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from random import randrange

class service_node(Node):
    def __init__(self):
        super().__init__("collection_service_node")
        self.srv = self.create_service(Location, "target", self.target_callback)
        self.pub_ = self.create_publisher(Float32MultiArray, "coordinates", 10)
        self.sub_ = self.create_subscription(String, "/soil_collection/status", self.callback_, 10)
        # creating a service for Location data
        # creating publisher to 'coordinates' to give current position
        # creating subscriber to 'soil collection status' to get data about whether soil collection was successful
        self.startx = 0
        self.starty = 0
        self.status = None
        self.targetx = 0
        self.targety = 0
        # creating start and target coordinates as parameters to use later
        self.get_logger().info("Collection service node started")

    def target_callback(self, request, response):
        if randrange(0, 2): # taking 50% probability of goal being an obstacle
            self.status = 1
        else:
            self.status = 0
        response.status = self.status
        self.targetx = request.target_x
        self.targety = request.target_y
        self.get_logger().info(f"Request received: ({request.target_x}, {request.target_y})")
        msg = Float32MultiArray()
        msg.data = [float(self.targetx), float(self.targety), float(self.status)]
        # publishing target position and collection status data to 'coordinates'
        # this is published only once to start the sub-pub loop of collection_service and rover_collection
        self.pub_.publish(msg)
        return response

    def callback_(self, string): 
        if string.data == "Collection Failed" or not randrange(0, 20):
            # collection can fail due to goal being an obstacle or taking 5% probability that battery runs out
            # in between and rover has to go back to starting point
            self.status = 0
            msg = Float32MultiArray()
            msg.data = [float(self.startx), float(self.starty), float(self.status)]
            self.pub_.publish(msg)
            self.get_logger().info("Going back to starting point")
        else:
            self.get_logger().info("Collection successful")
            msg = Float32MultiArray()
            msg.data = [float(self.targetx), float(self.targety), float(self.status)]
            # publishing target position and status to 'coordinates'
            self.pub_.publish(msg)
            self.status = 1

def main(args=None):
    rclpy.init(args=args)
    node = service_node()
    rclpy.spin(node)
    rclpy.shutdown()

