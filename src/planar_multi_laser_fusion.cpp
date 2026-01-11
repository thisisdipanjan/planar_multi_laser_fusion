#include "planar_multi_laser_fusion/planar_multi_laser_fusion.hpp"

MultiLaserFusion::MultiLaserFusion() : Node("Planar_multi_laser_fusion"){
    RCLCPP_INFO(this->get_logger(),"Planar fusion node Executing");
    total_lasers = this->declare_parameter<int>("planar_multi_laser_fusion.laser_topics.total_lasers", 1);

    for (int i=0; i<total_lasers; i++){
        this->declare_parameter("planar_multi_laser_fusion.laser_topics.topic"+ std::to_string(i), "/scan");
        laser_topics.push_back(this->get_parameter("planar_multi_laser_fusion.laser_topics.topic"+ std::to_string(i)).as_string());
    }
    init_subscribers();
    init_publishers();
}

void MultiLaserFusion::laser_fusion_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg){

}

void MultiLaserFusion::init_publishers(){
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
}

void MultiLaserFusion::init_subscribers(){

    for (int i=0; i<total_lasers; i++){
        subscriptions_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topics[i], 10, std::bind(&MultiLaserFusion::laser_fusion_callback, this, std::placeholders::_1)));
    }
}