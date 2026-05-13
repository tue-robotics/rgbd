#include "rgbd/client_ros.h"

namespace rgbd {

ClientROS::ClientROS(const rclcpp::Node::SharedPtr& node)
    : ClientROSBase(node ? node : rclcpp::Node::make_shared("rgbd_client_ros"))
{
}

ClientROS::~ClientROS() = default;

bool ClientROS::initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    if (!ClientROSBase::initialize(rgb_image_topic, depth_image_topic, cam_info_topic))
    {
        return false;
    }

    sync_->registerCallback(std::bind(&ClientROS::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    return true;
}

bool ClientROS::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    rclcpp::spin_some(node_);
    return new_image_;
}

ImagePtr ClientROS::nextImage()
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

}  // namespace rgbd
