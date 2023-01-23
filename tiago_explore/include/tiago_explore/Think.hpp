#ifndef TIAGO_EXPLORE__THINK_HPP_
#define TIAGO_EXPLORE__THINK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/world.hpp"
#include "std_msgs/msg/string.hpp"


class Think : public rclcpp::Node {

    public: 
        Think(); 

    private: 
        rclcpp::Subscription<custom_interfaces::msg::World>::SharedPtr world_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
        custom_interfaces::msg::World::UniquePtr world_; 
        
        void callback(custom_interfaces::msg::World::UniquePtr msg);
        std_msgs::msg::String think_policy(); 
        
}; 

#endif