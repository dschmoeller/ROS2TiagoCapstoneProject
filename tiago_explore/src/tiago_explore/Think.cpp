#include "tiago_explore/Think.hpp"

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/world.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define Constructor 
Think::Think()
: Node("think_node") {
    // Subscribe to a world model 
    world_sub_ = create_subscription<custom_interfaces::msg::World>(
        "world_model", rclcpp::SensorDataQoS(),
        std::bind(&Think::callback, this, _1));

    // Publish an action according to internal think policy 
    action_pub_ = create_publisher<std_msgs::msg::String>("action", 10); 
}

// React to incoming/updated world model 
void Think::callback(custom_interfaces::msg::World::UniquePtr msg){
  // Update current state of the world   
  world_ = std::move(msg);

  // Think 
  std_msgs::msg::String next_action = think_policy(); 

  // Publish identified action 
  action_pub_->publish(next_action); 
}

// Definition of think policy 
std_msgs::msg::String Think::think_policy(){
    // Move forward if there's not an obstacle ahead within 0.5 m  of distance
    // Otherwise, turn as long as the forward path is free again 
    std_msgs::msg::String action;
    bool center_blocked = world_->laser_distance_center < 1.0f;
    bool right_blocked = world_->laser_distance_right < 1.0f;
    bool left_blocked = world_->laser_distance_left < 1.0f;
    if (center_blocked || left_blocked ){
        action.data = "turn_right"; 
    } else {
        action.data = "forward"; 
    }
     
    return action;
}