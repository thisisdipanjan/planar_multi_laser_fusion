#ifndef PLANAR_MULTI_LASER_FUSION_HPP
#define PLANAR_MULTI_LASER_FUSION_HPP

#include "planar_multi_laser_fusion/utils.hpp"

class MultiLaserFusion : public rclcpp::Node{
    public:
        MultiLaserFusion();
        void init_subscribers();
        void init_publishers();

    private:
        bool tf_debug{false};
        void publish_tf();
        int total_lasers{};
        sensor_msgs::msg::LaserScan fused_msg;
        geometry_msgs::msg::TransformStamped tf;
        std::shared_ptr<tf2_ros::TransformListener>(tf_buffer_Scan1);
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster>static_tf_broadcaster;
        std::string laser_topics_1, laser_topics_2, laser_topics_3, laser_topics_4;
        geometry_msgs::msg::TransformStamped tf_scan1, tf_scan2, tf_scan3, tf_scan4; 
        void laser_fusion_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_scan1, tf_buffer_scan2, tf_buffer_scan3, tf_buffer_scan4;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laser_1, subscriber_laser_2,
                                                                     subscriber_laser_3, subscriber_laser_4;
};

#endif