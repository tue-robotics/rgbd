/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_BASE_H_
#define RGBD_CLIENT_ROS_BASE_H_

#include <rclcpp/rclcpp.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rgbd/types.h"

#include <memory>

namespace rgbd {

using RGBDApproxPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class ClientROSBase {
public:
    explicit ClientROSBase(const rclcpp::Node::SharedPtr& node);
    virtual ~ClientROSBase();

    bool initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);
    bool deinitialize();

    bool initialized() const { return static_cast<bool>(sync_); }

protected:
    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<message_filters::Synchronizer<RGBDApproxPolicy>> sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_rgb_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_sync_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
    image_geometry::PinholeCameraModel cam_model_;

    bool new_image_;
    Image* image_ptr_;

    void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg);
    bool imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg);
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_ROS_BASE_H_
