#ifndef RGBD_SERVER_ROS_H_
#define RGBD_SERVER_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rgbd/types.h"

#include <memory>

namespace rgbd {

class ServerROS {
public:
    explicit ServerROS(const rclcpp::Node::SharedPtr& node = nullptr);
    virtual ~ServerROS();

    void initialize(std::string ns = "", bool publish_rgb = false, bool publish_depth = false, bool publish_pc = false);
    void send(const Image& image);

protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
    rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZRGB>>::SharedPtr pub_depth_pc_;
};

}  // namespace rgbd

#endif  // RGBD_SERVER_ROS_H_
