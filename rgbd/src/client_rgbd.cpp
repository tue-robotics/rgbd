#include "rgbd/client_rgbd.h"

#include "rgbd/ros/conversions.h"

namespace rgbd {

ClientRGBD::ClientRGBD(const rclcpp::Node::SharedPtr& node)
    : node_(node ? node : rclcpp::Node::make_shared("rgbd_client_rgbd"))
    , new_image_(false)
    , image_ptr_(nullptr)
{
}

ClientRGBD::~ClientRGBD() = default;

bool ClientRGBD::initialize(const std::string& server_name)
{
    sub_image_ = node_->create_subscription<rgbd_interfaces::msg::RGBD>(
        server_name,
        rclcpp::SensorDataQoS(),
        std::bind(&ClientRGBD::rgbdImageCallback, this, std::placeholders::_1));
    return true;
}

bool ClientRGBD::deinitialize()
{
    sub_image_.reset();
    return true;
}

bool ClientRGBD::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    rclcpp::spin_some(node_);
    return new_image_;
}

ImagePtr ClientRGBD::nextImage()
{
    new_image_ = false;
    image_ptr_ = nullptr;
    rclcpp::spin_some(node_);
    if (!new_image_)
    {
        delete image_ptr_;
        return nullptr;
    }
    return ImagePtr(image_ptr_);
}

void ClientRGBD::rgbdImageCallback(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg)
{
    new_image_ = convert(msg, image_ptr_);
}

}  // namespace rgbd
