#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    // Initialize Ignition Transport node
    ignition::transport::Node node;

    // Topic to publish
    std::string topic = "/debug_cmd_vel";

    // Create a publisher for the topic
    auto pub = node.Advertise<ignition::msgs::Twist>(topic);
    if (!pub)
    {
        std::cerr << "Error creating publisher on topic: " << topic << std::endl;
        return -1;
    }

    std::cout << "Publishing to topic: " << topic << std::endl;

    // Create a message to send
    ignition::msgs::Twist msg;

    // Set the linear and angular velocities
    msg.mutable_linear()->set_x(0.0);  // Linear velocity in the x direction
    msg.mutable_linear()->set_y(0.0);
    msg.mutable_linear()->set_z(0.0);
    msg.mutable_angular()->set_x(0.0);
    msg.mutable_angular()->set_y(0.0);
    msg.mutable_angular()->set_z(0.5);  // Angular velocity around z-axis

    // Publish the message at regular intervals
    while (true)
    {
        pub.Publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return 0;
}
