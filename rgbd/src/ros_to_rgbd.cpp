#include <rclcpp/rclcpp.hpp>

#include "rgbd/client_ros.h"
#include "rgbd/image.h"
#include "rgbd/server.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ros_to_rgbd");

    double rate = node->declare_parameter<double>("rate", 30.0);

    rgbd::RGBStorageType rgb_type;
    std::string rgb_type_str = node->declare_parameter<std::string>("rgb_storage", "lossless");
    if (rgb_type_str == "none")
        rgb_type = rgbd::RGB_STORAGE_NONE;
    else if (rgb_type_str == "lossless")
        rgb_type = rgbd::RGB_STORAGE_LOSSLESS;
    else if (rgb_type_str == "jpg")
        rgb_type = rgbd::RGB_STORAGE_JPG;
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown 'rgb_storage' type: should be 'none', 'lossless', or 'jpg'.");
        return 1;
    }

    rgbd::DepthStorageType depth_type;
    std::string depth_type_str = node->declare_parameter<std::string>("depth_storage", "lossless");
    if (depth_type_str == "none")
        depth_type = rgbd::DEPTH_STORAGE_NONE;
    else if (depth_type_str == "lossless")
        depth_type = rgbd::DEPTH_STORAGE_LOSSLESS;
    else if (depth_type_str == "png")
        depth_type = rgbd::DEPTH_STORAGE_PNG;
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown 'depth_storage' type: should be 'none', 'lossless', or 'png'.");
        return 1;
    }

    rgbd::ClientROS client(node);
    rgbd::Server server(node);

    client.initialize("rgb_image", "depth_image", "cam_info");
    server.initialize("rgbd", rgb_type, depth_type);

    rgbd::ImagePtr image_ptr;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        image_ptr = client.nextImage();
        if (image_ptr)
            server.send(*image_ptr);
        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
