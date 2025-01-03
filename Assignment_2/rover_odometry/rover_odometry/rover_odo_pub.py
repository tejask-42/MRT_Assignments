#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from random import uniform
from geometry_msgs.msg import Twist
from mars_msgs.msg import RoverOdometry 


class odo_data(Node):
    def __init__(self):
        super().__init__("Odometry_data_publisher")
        self.pub_ = self.create_publisher(RoverOdometry, "odo_topic", 10)
        self.timer_ = self.create_timer(1, self.odo_pub)
        self.get_logger().info("Odometry publisher node started")
        # creating a publisher which sends RoverOdometry data to 'odo_topic' every second


    def odo_pub(self):
        msg = RoverOdometry()
        tw = Twist()
        msg.rover_id = 1
        if msg.orientation: # if orientation is initialised, increment by 0.01 else initialise to 0.5
            msg.orientation += 0.01
        else:
            msg.orientation = 0.5
        tw.linear.x = uniform(-3, 3) # linear velocity in x and y can take any float value from -3 to 3 m/s
        tw.linear.y = uniform(-3, 3)
        msg.linear_velocity = tw
        msg.angular_velocity = uniform(0, 1)
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = odo_data()
    rclpy.spin(node)
    rclpy.shutdown()
