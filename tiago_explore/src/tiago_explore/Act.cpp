#include "tiago_explore/Act.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define Constructor 
Act::Act()
: Node("act_node") {
    // Subscribe to an action string 
    action_sub_ = create_subscription<std_msgs::msg::String>(
        "action", rclcpp::SensorDataQoS(),
        std::bind(&Act::callback, this, _1));

    // Publish a motion command to robot interface 
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("motion_command", 10); 
}

// React to incoming action strings
void Act::callback(std_msgs::msg::String::UniquePtr msg){
  // Transform action message into actual robot motion command 
  geometry_msgs::msg::Twist robot_motion_command; 
  if (msg->data == "forward"){
    robot_motion_command.linear.x = 0.3f;
  }
  else if (msg->data == "turn_right"){
    robot_motion_command.angular.z = -0.6f;
  }
  // Publish robot motion command
  command_pub_->publish(robot_motion_command); 
}