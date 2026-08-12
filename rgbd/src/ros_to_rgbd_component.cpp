#include "rgbd/ros_to_rgbd_component.h"
#include "rgbd/client_ros.h"
#include "rgbd/image.h"
#include "rgbd/server.h"
#include "rgbd/types.h"

#include <memory>
#include <rcl_interfaces/msg/detail/floating_point_range__struct.hpp>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>

#include <chrono>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <stdexcept>

namespace rgbd
{

RosToRGBDComponent::RosToRGBDComponent(const rclcpp::NodeOptions& options) :
    rclcpp::Node("ros_to_rgbd", options),
    rgb_type_(parseRGBStorageType(declare_parameter<std::string>("rgb_storage", "lossless"))),
    depth_type_(parseDepthStorageType(declare_parameter<std::string>("depth_storage", "lossless")))

{
    rcl_interfaces::msg::ParameterDescriptor rate_descriptor;
    rate_descriptor.description = "Processing loop rate in Hz";
    rcl_interfaces::msg::FloatingPointRange rate_range;
    rate_range.from_value = 0.01;
    rate_range.to_value = std::numeric_limits<double>::max();
    rate_range.step = 0.0;
    rate_descriptor.floating_point_range.push_back(rate_range);

    double rate = declare_parameter<double>("rate", 30.0, rate_descriptor);
    if (rate <= 0.0)
    {
        RCLCPP_WARN(get_logger(), "Parameter 'rate' must be > 0, defaulting to 30Hz");
        rate = 30.0;
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, [this] { runOnce(); });
}

RGBStorageType RosToRGBDComponent::parseRGBStorageType(const std::string& rgb_type_str)
{
    if (rgb_type_str == "none")
        return RGBStorageType::RGB_STORAGE_NONE;
    if (rgb_type_str == "lossless")
        return RGBStorageType::RGB_STORAGE_LOSSLESS;
    if (rgb_type_str == "jpg")
        return RGBStorageType::RGB_STORAGE_JPG;

    throw std::invalid_argument("Unknown 'rgb_storage' type: should be 'none', 'lossless', or 'jpg'.");
}

DepthStorageType RosToRGBDComponent::parseDepthStorageType(const std::string& depth_type_str)
{
    if (depth_type_str == "none")
        return DepthStorageType::DEPTH_STORAGE_NONE;
    if (depth_type_str == "lossless")
        return DepthStorageType::DEPTH_STORAGE_LOSSLESS;
    if (depth_type_str == "png")
        return DepthStorageType::DEPTH_STORAGE_PNG;

    throw std::invalid_argument("Unknown 'depth_storage' type: should be 'none', 'lossless', or 'png'.");
}

bool RosToRGBDComponent::initializeInterfaces()
{
    auto node = shared_from_this();
    client_ = std::make_unique<ClientROS>(node);
    server_ = std::make_unique<Server>(node);

    if (!client_->initialize("rgb_image", "depth_image", "cam_info"))
    {
        RCLCPP_ERROR(get_logger(), "Failed to initialize ClientROS");
        return false;
    }

    server_->initialize("rgbd", rgb_type_, depth_type_);
    return true;
}

void RosToRGBDComponent::runOnce()
{
    if (!interfaces_initialized_)
    {
        interfaces_initialized_ = initializeInterfaces();
        if (!interfaces_initialized_)
            return;
    }

    const ImagePtr image_ptr = client_->nextImage();
    if (image_ptr)
        server_->send(*image_ptr);
}

} // namespace rgbd
