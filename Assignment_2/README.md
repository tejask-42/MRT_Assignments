# Mars Rover Simulation System

This repository contains a ROS 2-based simulation system showcasing the Mars rover's core functionalities. It demonstrates the following key tasks:

## Tasks

### 1. Rover Status Monitoring
Publishes rover status information, including battery level, temperature, and health status.

### 2. Odometry Simulation
Simulates the rover's odometry, providing data on position, orientation, and velocity by using custom message types.

### 3. Navigation with Obstacle Avoidance
Implements shortest path navigation in a grid environment while avoiding obstacles.

### 4. Soil Collection System
Simulates soil sample collection at specified coordinates with obstacle handling.

## Usage

1. Clone and build the repository:
   ```bash
   git clone https://github.com/tejask-42/mars-rover-simulation.git
   cd mars-rover-simulation
   colcon build --symlink-install
   source install/setup.bash
