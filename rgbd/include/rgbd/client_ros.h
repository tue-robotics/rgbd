/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_H_
#define RGBD_CLIENT_ROS_H_

#include <rclcpp/rclcpp.hpp>

#include "rgbd/client_ros_base.h"
#include "rgbd/types.h"

namespace rgbd {

class ClientROS : public ClientROSBase {
public:
    explicit ClientROS(const rclcpp::Node::SharedPtr& node = nullptr);
    ~ClientROS() override;

    bool initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);
    bool nextImage(Image& image);
    ImagePtr nextImage();
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_ROS_H_
