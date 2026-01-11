#include "planar_multi_laser_fusion/planar_multi_laser_fusion.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLaserFusion>());
    MultiLaserFusion fusion;
    fusion.tf_debug = true; //To print tf from base_link to scan frames
    rclcpp::shutdown();
    return 0;
}