#include "planar_multi_laser_fusion/planar_multi_laser_fusion.hpp"

int main(int argc, char* argv[]){
    MultiLaserFusion fusion;
    fusion.tf_debug = true; //To print tf from base_link to scan frames

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLaserFusion>());
    rclcpp::shutdown();
    return 0;
}