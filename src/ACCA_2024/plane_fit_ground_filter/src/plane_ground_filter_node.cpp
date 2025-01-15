#include "plane_ground_filter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaneGroundFilter>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
