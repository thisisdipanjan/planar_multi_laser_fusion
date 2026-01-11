#include "planar_multi_laser_fusion/planar_multi_laser_fusion.hpp"

MultiLaserFusion::MultiLaserFusion() : Node("Planar_multi_laser_fusion"){
    RCLCPP_INFO(this->get_logger(),"Planar fusion node Executing");
    total_lasers = this->declare_parameter<int>("planar_multi_laser_fusion.laser_topics.total_lasers", 1);

    this->declare_parameter("planar_multi_laser_fusion.laser_topics.topic1", "/scan1");
    laser_topics_1 = this->get_parameter("planar_multi_laser_fusion.laser_topics.topic1").as_string();

    tf_buffer_scan1 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_scan2 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_scan3 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_scan4 = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_buffer_scan1->lookupTransform("base_link", "scan1", tf2::TimePointZero);
    tf_buffer_scan2->lookupTransform("base_link", "scan2", tf2::TimePointZero);
    tf_buffer_scan3->lookupTransform("base_link", "scan3", tf2::TimePointZero);
    tf_buffer_scan4->lookupTransform("base_link", "scan4", tf2::TimePointZero);
    
    init_subscribers();
    init_publishers();
}

void MultiLaserFusion::publish_tf(){
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "scan";

    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);

    tf.transform.rotation.x = q.getX();
    tf.transform.rotation.y = q.getY();
    tf.transform.rotation.z = q.getZ();
    tf.transform.rotation.w = q.getW();

    static_tf_broadcaster->sendTransform(tf);
}

void MultiLaserFusion::laser_fusion_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg){
    /*
    scan2 --------- scan1
    |                   |
    |                   |
    |     base_link     |
    |                   |
    |                   |
    |                   |
    scan3 --------- scan4
    
    */
    


    const std::string &frame = laser_msg->header.frame_id;
    if (frame == "scan1"){
        
    }

    else if (frame == "scan2"){
        ;
    }

    else if (frame == "scan3"){
        ;
    }

    else if (frame== "scan4"){
        ;
    }

    else{
        RCLCPP_WARN(this->get_logger(), "Topic nae mismatch, check params");
    }

    publisher_->publish(fused_msg);
}

void MultiLaserFusion::init_publishers(){
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
}

void MultiLaserFusion::init_subscribers(){
    subscriber_laser_1 = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topics_1, 10, std::bind(&MultiLaserFusion::laser_fusion_callback, this, std::placeholders::_1));
    subscriber_laser_2 = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topics_2, 10, std::bind(&MultiLaserFusion::laser_fusion_callback, this, std::placeholders::_1));
    subscriber_laser_3 = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topics_3, 10, std::bind(&MultiLaserFusion::laser_fusion_callback, this, std::placeholders::_1));
    subscriber_laser_4 = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topics_4, 10, std::bind(&MultiLaserFusion::laser_fusion_callback, this, std::placeholders::_1));
}