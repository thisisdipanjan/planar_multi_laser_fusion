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

    tf_scan1 = tf_buffer_scan1->lookupTransform("base_link", "scan1", tf2::TimePointZero);
    tf_scan2 = tf_buffer_scan1->lookupTransform("base_link", "scan2", tf2::TimePointZero);
    tf_scan3 = tf_buffer_scan1->lookupTransform("base_link", "scan3", tf2::TimePointZero);
    tf_scan4 = tf_buffer_scan1->lookupTransform("base_link", "scan4", tf2::TimePointZero);
    
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
    /* This works in ideal condition, need to add more flexibility after testing code.
    scan2 --------- scan1
    |                   |
    |                   |
    |     base_link     |
    |                   |
    |                   |
    |                   |
    scan3 --------- scan4
    
    */
    CartesianCoordinates scan1, scan2, scan3, scan4;
    if (tf_debug){
        scan1.x = tf_scan1.transform.translation.x;
        scan1.y = tf_scan1.transform.translation.y;
        RCLCPP_INFO(this->get_logger(), "");
    }


    double index = laser_msg->ranges.size();
    const std::string &frame = laser_msg->header.frame_id;
    for (int i=index/3; i<=index/2; i++){
        if (frame == "scan1"){
            fused_msg.ranges = laser_msg->ranges;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Topic name mismatch [scan1], check params");
        }
    }

    for (int i=index/4; i<=index/3; i++){
        if (frame == "scan2"){
            fused_msg.ranges = laser_msg->ranges;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Topic name mismatch [scan2], check params");
        }
    }

    for (int i=0; i<=index/4; i++){
        if (frame == "scan3"){
            fused_msg.ranges = laser_msg->ranges;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Topic name mismatch [scan3], check params");
        }
    }

    for (int i=index/2; i<=index; i++){
        if (frame == "scan1"){
            fused_msg.ranges = laser_msg->ranges;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Topic name mismatch [scan4], check params");
        }
    }

    publisher_->publish(fused_msg);
    publish_tf();
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