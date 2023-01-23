#include "tiago_explore/See.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/world.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define Constructor 
See::See()
: Node("see_node") {
    // Subscribe to lidar
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "input_scan", rclcpp::SensorDataQoS(),
        std::bind(&See::lidar_callback, this, _1));

    // Publish a representation of the world 
    world_pub_ = create_publisher<custom_interfaces::msg::World>("world_model", 10); 
    timer_ = create_wall_timer(100ms, 
        std::bind(&See::publish_cycle, this)); 
}

// React to incoming lidar messages
void See::lidar_callback(sensor_msgs::msg::LaserScan::UniquePtr msg){
  //RCLCPP_INFO(get_logger(), "Hello from the lidar callback");
  lidar_message_ = std::move(msg);
}

// Publishing a representation of the world
void See::publish_cycle(){
  
   // Do nothing until messages arrive
  if (lidar_message_ == nullptr) {
    return;
  }

  // Time sync sensor modalities 
  // TODO: Once other messages are added 
  
  // Extract relevant information from laser messages
  custom_interfaces::msg::World world_repr; 
  int pos = lidar_message_->ranges.size()/2;
  world_repr.laser_distance_center = lidar_message_->ranges[pos]; 
  world_repr.laser_distance_left = lidar_message_->ranges[pos + 200]; 
  world_repr.laser_distance_right = lidar_message_->ranges[pos - 200];   
  
  // Publish 
  world_pub_->publish(world_repr); 
}