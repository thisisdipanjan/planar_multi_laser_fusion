#ifndef PLANAR_MULTI_LASER_FUSION_HPP
#define PLANAR_MULTI_LASER_FUSION_HPP

#include "planar_multi_laser_fusion/utils.hpp"

class MultiLaserFusion : public rclcpp::Node{
    public:
        MultiLaserFusion();
        void init_subscribers();
        void init_publishers();

    private:
        int total_lasers{};
        std::vector<std::string> laser_topics;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
        void laser_fusion_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
};

#endif