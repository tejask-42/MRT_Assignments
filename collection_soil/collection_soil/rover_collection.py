#!/usr/bin/env python3
from mars_srv.srv import Location
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class rover_collection(Node):
    def __init__(self):
        super().__init__("rover_collection_node")
        self.sub_ = self.create_subscription(Float32MultiArray, "coordinates", self.callback_, 10)
        self.pub_ = self.create_publisher(String, "/soil_collection/status", 10)
        # creating a subscriber to 'coordinates' to get target position data and status
        # creating a publisher to 'soil collection status' to publish collection status
        self.sub_
        self.get_logger().info("Rover collection node started")
        self.x = None
        self.y = None
        # creating x and y parameters to use later


    def callback_(self, array):
        if self.x == array.data[0] and self.y == array.data[1] or (not (array.data[0] and array.data[1])):
            pass
            # if (X, y) value is same as what we are getting again, it means we havent got new input yet and 
            # pub-sub loop is running on older input
            # if array data is both (0, 0) it means we are at starting position so no need to log collection failed again
        else:
            status = int(array.data[2])
            if status:
                msg = String()
                msg.data = "Collection Successful"
                self.pub_.publish(msg)
                self.get_logger().info(f"Successfully collected samples at ({array.data[0]}, {array.data[1]})")
            else:
                msg = String()
                msg.data = "Collection Failed"
                self.pub_.publish(msg)
                self.get_logger().warn(f"Failed to collect samples at ({array.data[0]}, {array.data[1]})")
            self.x = array.data[0]
            self.y = array.data[1]


def main(args=None):
    rclpy.init(args=args)
    node = rover_collection()
    rclpy.spin(node)
    rclpy.shutdown()