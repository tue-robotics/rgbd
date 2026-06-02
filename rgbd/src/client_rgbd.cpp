#include "rgbd/client_rgbd.h"

#include "rgbd/ros/conversions.h"

namespace rgbd {

ClientRGBD::ClientRGBD(const rclcpp::Node::SharedPtr& node)
    : node_(node ? node : rclcpp::Node::make_shared("rgbd_client_rgbd"))
    , cb_group_image_(nullptr)
    , new_image_(false)
    , image_ptr_(nullptr)
{
}

ClientRGBD::~ClientRGBD() = default;

bool ClientRGBD::initialize(const std::string& server_name)
{
    cb_group_image_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_image_;
    sub_image_ = node_->create_subscription<rgbd_interfaces::msg::RGBD>(
        server_name,
        rclcpp::SensorDataQoS(),
        std::bind(&ClientRGBD::rgbdImageCallback, this, std::placeholders::_1),
        sub_options);
    executor_image_.add_callback_group(cb_group_image_, node_->get_node_base_interface());
    return true;
}

bool ClientRGBD::deinitialize()
{
    executor_image_.remove_callback_group(cb_group_image_);
    cb_group_image_.reset();
    sub_image_.reset();
    return true;
}

bool ClientRGBD::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    executor_image_.spin_some();
    return new_image_;
}

ImagePtr ClientRGBD::nextImage()
{
    new_image_ = false;
    image_ptr_ = nullptr;
    executor_image_.spin_some();
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
