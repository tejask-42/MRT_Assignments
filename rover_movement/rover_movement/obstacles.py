#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray, String

class obstacle_node(Node):
    def __init__(self):
        super().__init__("Obstacle_Avoidance_Node")
        self.sub_1 = self.create_subscription(Float32MultiArray, "/obstacles", self.callback_1, 10)
        self.pub_ = self.create_publisher(Float32MultiArray, "/obstacle_coordinates", 10)
        self.sub_ = self.create_subscription(String, "/navigation/status", self.callback_,10)
        self.timer_ = self.create_timer(1, self.publish_obs)
        self.get_logger().info("Obstacle avoidance node started")
        # creating subscription to 'obstacles' to get obstacle data from map_pub
        # creating publisher to give changed map with current position as origin to 'obstacle coordinates'
        # creating subscription to 'navigation status' to get current position and number of steps
        self.sub_1
        self.sub_
        self.array = []
        self.x = None 
        self.y = None
        self.gx = None
        self.gy = None
        self.n = None
        self.m = None

    def callback_1(self, array):
        self.array = array.data
        self.get_logger().info("Received map data")
        self.n, self.m, self.x, self.y, self.gx, self.gy = [int(i) for i in self.array[-6:]]
        # unpacking grid dimesions, start, end into class parameters

    
    def publish_obs(self):
        # constructing an array containing positions of obstacles with current position as origin
        if self.array:
            obs_list = []
            for i in range(self.n):
                for j in range(self.m):
                    if self.array[i*self.m + j]:
                        obs_list.append(float(i - self.x))
                        obs_list.append(float(j - self.y))
            obs_list.extend(self.array[-6:]) # again giving dimesions, start, end data at the end
            msg = Float32MultiArray()
            msg.data = obs_list
            self.pub_.publish(msg)
        
    
    def callback_(self, string):
        # string of the form "(a, b) steps"
        if string.data[:9] == "Completed":
            self.get_logger().info("Goal Reached!")
            self.get_logger().info(f"{string.data[9:]} Steps")
            rclpy.shutdown()
        else:
            x, y, steps = string.data.split()
            self.x = int(x[1:-1]) # formatting string to get value of a, b, steps
            self.y = int(y[:-1])
            self.steps = int(steps)
            self.get_logger().info(f"Step {self.steps}: ({self.y}, {self.x})")


def main(args=None):
    rclpy.init(args=args)
    node = obstacle_node()
    rclpy.spin(node)
    rclpy.shutdown()