#ifndef TIAGO_EXPLORE__SEE_HPP_
#define TIAGO_EXPLORE__SEE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/world.hpp"

class See : public rclcpp::Node {

    public: 
        See(); 

    private: 
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        sensor_msgs::msg::LaserScan::UniquePtr lidar_message_;
        rclcpp::Publisher<custom_interfaces::msg::World>::SharedPtr world_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void lidar_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
        void publish_cycle(); 
}; 

#endif