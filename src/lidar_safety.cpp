#include "safety_field/lidar_safety.hpp"
/*
Author: Dipanjan Maji
Module: Lidar Safety
Desription: Lidar Safety module when used will produce messages "SAFE_ZONE", "STOP_ZONE", "WARNING_ZONE" on topic /lidar_safety_filed using /scan topic.
*/

LidarSafety::LidarSafety(): Node("LidarSafety"){
    x_coordinate = this->declare_parameter("safety_field.x_coordinate", 1.0);
    y_coordinate = this->declare_parameter("safety_field.y_coordinate", 1.0);
    

    RCLCPP_INFO(this->get_logger(), "Params loaded and Lidar Safety node Executing");
    init_subscribers();
    init_publishers();
}


double LidarSafety::mid_distance(double x_a, double y_a, double x_b, double y_b){
    double midpoint_x = (x_a + x_b)/2;
    double midpoint_y = (y_a + y_b)/2;
    return reference_height = sqrt(pow((midpoint_x-0),2) + pow((midpoint_y-0),2));
}

void LidarSafety::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg){
    if (laser_msg){
        double laserdata_size = laser_msg->ranges.size();
        std::vector<double> angle;

        for(double i=0; i<laserdata_size; i++){
            angle.push_back(laser_msg->angle_min + i*laser_msg->angle_increment);
        }


        for(double i =0; i<angle.size(); i++){
            //positive X
            if (angle[i] >= 270 and angle[i] < 90){ 
                reference_height = mid_distance(x_coordinate, y_coordinate, x_coordinate, -y_coordinate);
            }
            //positive Y
            else if (angle[i] >= 0 and angle[i] < 180){
                reference_height = mid_distance(-x_coordinate, y_coordinate, x_coordinate, y_coordinate);
            }
            //negative X
            else if (angle[i] >= 90 and angle[i] < 270){ 
                reference_height = mid_distance(-x_coordinate, y_coordinate, -x_coordinate, -y_coordinate);
            }
            //negative Y
            else if (angle[i] >= 180 and angle[i] < 360){ 
                reference_height = mid_distance(-x_coordinate, -y_coordinate, x_coordinate, -y_coordinate);
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Params mismatch, please check calculation");
            } 

            sweep_safety_distance = reference_height/cos(angle[i]);
            if (laser_msg->ranges[i] >= sweep_safety_distance*safetyMultipler){
                safety_field_message.data = "SAFE_ZONE";
            }
                        
            if (laser_msg->ranges[i] <= sweep_safety_distance){
                safety_field_message.data = "STOP_ZONE";
            }

            if (laser_msg->ranges[i] == sweep_safety_distance){
                safety_field_message.data = "WARNING_ZONE";
            }
        }
    }
    else{
        std::cout << "NO message in /scan topic, check if its online" << std::endl;
    }
    publisher_->publish(safety_field_message);
}


void LidarSafety::init_subscribers(){
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LidarSafety::lidarCallback, this, std::placeholders::_1));
}

void LidarSafety::init_publishers(){
    publisher_ = this->create_publisher<std_msgs::msg::String>("lidar_safety_message", 10);
}