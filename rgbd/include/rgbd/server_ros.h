#ifndef RGBD_SERVER_ROS_H_
#define RGBD_SERVER_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rgbd/types.h"

#include <memory>

namespace rgbd
{

/**
 * @brief Server which publishes ROS rgb image, depth image and pointcloud messages
 */
class ServerROS
{
  public:
    /**
     * @brief Constructor
     */
    explicit ServerROS(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     */
    virtual ~ServerROS();

    /**
     * @brief initialize server
     * @param ns relative or absolute namespace of publishers
     * @param publish_rgb Publish rgb image and camera info
     * @param publish_depth Publish depth image and camera info
     * @param publish_pc Publish point cloud
     */
    void initialize(std::string ns = "", bool publish_rgb = false, bool publish_depth = false, bool publish_pc = false);

    /**
     * @brief Publish a new image to the selected ROS topics
     * @param image Image to be published
     */
    void send(const Image& image);

  protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_pc_;
};

} // namespace rgbd

#endif // RGBD_SERVER_ROS_H_
