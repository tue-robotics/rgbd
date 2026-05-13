/**
 * This client listens to RGBD messages on a single topic directing rgbd::Image.
 */

#ifndef RGBD_CLIENT_RGBD_H_
#define RGBD_CLIENT_RGBD_H_

#include <rclcpp/rclcpp.hpp>

#include <rgbd_interfaces/msg/rgbd.hpp>

#include "rgbd/types.h"

namespace rgbd {

class ClientRGBD {
public:
    explicit ClientRGBD(const rclcpp::Node::SharedPtr& node = nullptr);
    virtual ~ClientRGBD();

    bool initialize(const std::string& server_name);
    bool deinitialize();

    bool initialized() const { return static_cast<bool>(sub_image_); }

    bool nextImage(Image& image);
    ImagePtr nextImage();

protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rgbd_interfaces::msg::RGBD>::SharedPtr sub_image_;

    bool new_image_;
    Image* image_ptr_;

    void rgbdImageCallback(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg);
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_RGBD_H_
