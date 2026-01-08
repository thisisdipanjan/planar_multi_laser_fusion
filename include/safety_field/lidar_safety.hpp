#ifndef LIDAR_SAFETY_HPP
#define LIDAR_SAFETY_HPP


#include <cmath>
#include <vector>
#include <rclcpp/node.hpp>
#include<rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarSafety : public rclcpp::Node{
    public:
        LidarSafety();
        void init_subscribers();
        void init_publishers();
        float safetyMultipler{1.2};
        std_msgs::msg::String safety_field_message;
    private:
        double x_coordinate;
        double y_coordinate;
        double reference_height;
        double sweep_safety_distance;
        double mid_distance(double, double, double, double);
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
};

#endif