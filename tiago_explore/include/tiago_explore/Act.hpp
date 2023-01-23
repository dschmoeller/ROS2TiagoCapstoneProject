#ifndef TIAGO_EXPLORE__ACT_HPP_
#define TIAGO_EXPLORE__ACT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Act : public rclcpp::Node {

    public: 
        Act(); 

    private: 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_; 
        
        void callback(std_msgs::msg::String::UniquePtr msg);        
}; 

#endif