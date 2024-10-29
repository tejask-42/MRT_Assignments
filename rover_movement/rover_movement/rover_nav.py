#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray, String
isvalid = lambda x, y, array, visited: 0 <= x < len(array[0]) and 0 <= y < len(array) and not array[y][x] and not visited[y][x]
# creating a function to check if a given point is inside grid bounds and is not already visited
dirxns = [(-1, 0), (0, 1), (0, -1), (1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] # all 8 directions
def shortest_path(array, start, end):
    n = len(array)
    m = len(array[0])
    visited = [[False] * m for i in range(n)]
    
    queue = [(start[0], start[1], 0)]  # (x, y, distance)
    visited[start[1]][start[0]] = True
    predecessors = {start: None}  # To track the path
    
    while queue:
        x, y, dist = queue.pop(0)  
        
        # Check if we have reached the destination
        if (x, y) == end: # the first time (x, y) becomes equal to end has to be the shortest path
            # Reconstruct the path from end to start
            path = []
            while (x, y) is not None:
                path.append((x, y))
                if predecessors[(x, y)]:
                    x, y = predecessors[(x, y)]
                else:
                    break
            path.reverse()  # Reverse to get the path from start to end
            return dist, path
        
        # Explore all 8 possible directions
        for dx, dy in dirxns:
            nx, ny = x + dx, y + dy
            if isvalid(nx, ny, array, visited):
                visited[ny][nx] = True
                predecessors[(nx, ny)] = (x, y)  # Track where we came from
                queue.append((nx, ny, dist + 1))
    
    # If the end is unreachable, return -1 and an empty path
    return -1, []

    
def make_grid(obs_list, x, y, n, m): # use obstacle coordinates data and grid dimensions to construct obstacle map
    grid = [[0 for i in range(m)] for j in range(n)]
    for i in range(0, len(obs_list)-6, 2):
        a = int(x + obs_list[i])
        b = int(y + obs_list[i + 1])
        grid[b][a] = 1 # y increases downwards as (0, 0) is top left point
    return grid

class rover_node(Node):
    def __init__(self):
        super().__init__("Rover_Navigation_Node")
        self.sub_ = self.create_subscription(Float32MultiArray, "/obstacle_coordinates", self.callback_, 10)
        self.pub_ = self.create_publisher(String, "/navigation/status", 10)
        self.timer_ = self.create_timer(1, self.publish_data)
        self.sub_
        # creating subscription to 'obstacle_coordinates' topic to get obstacle list from obstacle_node
        # publishing current position and steps as a string to 'navigation status' topic
        self.get_logger().info("Rover navigation node started")
        self.x = None
        self.y = None
        self.steps = 0
        self.path = None
        self.gx = None
        self.gy = None
        self.n = 0
        self.m = 0
        self.first = True

    def callback_(self, array):
        self.n, self.m, self.x, self.y, self.gx, self.gy = [int(i) for i in array.data[-6:]]
        # taking the appended info about grid and start, end from array
        array = make_grid(array.data, self.x, self.y, self.n, self.m)
        self.get_logger().info("Received obstacle coordinates")
        if self.first and array[self.gy][self.gx]:
            self.get_logger().info("Invalid coordinates: goal is an obstacle.")
        if self.path == None:
            dist, path = shortest_path(array, (self.x, self.y), (self.gx, self.gy))
            if dist == -1:
                self.get_logger().info("Path not possible")
            else:
                self.path = path


    def publish_data(self):
        if self.first and self.path != None:
            self.path.pop(0)
            self.first = False
        msg = String()
        if self.path == []:
            msg.data = "Completed" + str(self.steps)
            self.pub_.publish(msg)
            # one by one we are taking out elements from path and sending to navigation status topic
            # once path is empty we are done
        if self.path != None:
            # start only if self.path is initialised
            self.steps += 1
            try:
                self.x, self.y = self.path.pop(0)
                msg.data = f"({self.x}, {self.y}) {self.steps}"
                self.pub_.publish(msg)
            except IndexError:
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = rover_node()
    rclpy.spin(node)
    rclpy.shutdown()
