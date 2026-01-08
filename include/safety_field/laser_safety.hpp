#include "safety_field.hpp"

class LaserSafety : public rclcpp::Node{
    public:
        LaserSafety();
        bool connection();
        bool warning_zone();
        bool stop_zone();
        bool safe_zone();
        bool stop_safety();
};
