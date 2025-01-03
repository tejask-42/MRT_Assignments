#!/usr/bin/env python3
import rclpy    
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from mars_msgs.msg import RoverOdometry 

class odometry_sub(Node):
    def __init__(self):
        super().__init__("receive_odometry_data")
        self.sub_ = self.create_subscription(RoverOdometry, "odo_topic", self.callback_, 10)
        self.get_logger().info("Odometry subscriber node started")
        self.sub_
        self.x = 0
        self.y = 0
        self.angle = 0
        self.vel = 0
        # creating a subscription to 'odo_topic' and creating parameters to store odometry data
    
    def callback_(self, rvrodo):
        self.x += rvrodo.linear_velocity.linear.x
        self.y += rvrodo.linear_velocity.linear.y
        self.angle = rvrodo.orientation
        self.angle += rvrodo.angular_velocity
        # updating odometry values every time new data comes in
        self.vel = pow(rvrodo.linear_velocity.linear.x ** 2 + rvrodo.linear_velocity.linear.y ** 2, 0.5)
        if self.vel > 3:
            self.get_logger().info("WARNING: Rover speed exceeded 3m/s")
            # checking if rover velocity has exceeded a threshold
        self.get_logger().info(f"Position: ({self.x:.2f}, {self.y:.2f})") # limiting to 2 decimal places for readability
        self.get_logger().info(f"Angle: {self.angle:.2f} rad")
        self.get_logger().info(f"Linear velocity: {self.vel:.2f}m/s\nAngular velocity: {rvrodo.angular_velocity:.2f}rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = odometry_sub()
    rclpy.spin(node)
    rclpy.shutdown()
