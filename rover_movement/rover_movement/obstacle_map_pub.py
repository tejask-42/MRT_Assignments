#!/usr/bin/env python3
import rclpy    
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray

class obstacle_map(Node):
    def __init__(self):
        super().__init__("create_obstacle_map")
        self.pub_ = self.create_publisher(Float32MultiArray, "/obstacles", 10)
        self.timer_ = self.create_timer(30, self.publish_data)
        self.get_logger().info("Obstacle map node started")
        # create an array to hold data about obstacle coordinates, published every 30 seconds

    def publish_data(self):
        n = int(input("Enter number of rows of map: "))
        m = int(input("Enter number of columns of map: "))
        array = []
        for i in range(n):
            for j in range(m):
                x = int(input(f"Enter value of ({j}, {i}) point: "))
                array.append(float(x))
        # taking input from user to make obstacle map, then printing map
        self.get_logger().info(f"{n}x{m} map produced: ")
        self.get_logger().info(str([array[k*m:(k + 1)*m] for k in range(n)]))
        self.x = int(input("Enter starting x coordinate: "))
        self.y = int(input("Enter starting y coordinate: "))
        self.gx = int(input("Enter goal x coordinate: "))
        self.gy = int(input("Enter goal y coordinate: "))
        # taking info about start, end coordinates
        self.n = n
        self.m = m
        array.append(float(n))
        array.append(float(m))
        array.append(float(self.x))
        array.append(float(self.y)) # adding data about grid dimensions, start, end at the end of obstacle grid
        array.append(float(self.gx))
        array.append(float(self.gy))
        if array[self.gy * m + self.gx]:
            self.get_logger().info("Invalid coordinates, goal is an obstacle")
            self.publish_data()
        msg = Float32MultiArray()
        # publishing obstacle grid with other data to 'obstaces' topic
        msg.data = array
        self.pub_.publish(msg)
        rclpy.shutdown()

        
def main(args=None):
    rclpy.init(args=args)
    node = obstacle_map()
    rclpy.spin(node)
    rclpy.shutdown()
