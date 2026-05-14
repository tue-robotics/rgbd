#include <rclcpp/rclcpp.hpp>

#include "rgbd/ros_to_rgbd_component.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rgbd::RosToRGBDComponent>(rclcpp::NodeOptions());
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
