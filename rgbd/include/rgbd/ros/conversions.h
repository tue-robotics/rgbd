#ifndef RGBD_ROS_CONVERSIONS_H_
#define RGBD_ROS_CONVERSIONS_H_

#include <rgbd_interfaces/msg/rgbd.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rgbd/image.h"

namespace cv
{
class Mat;
}
namespace geo
{
class DepthCamera;
}

namespace rgbd
{

bool convert(const cv::Mat& image, sensor_msgs::msg::Image& image_msg);

bool convert(const cv::Mat& image, const geo::DepthCamera& cam_model, sensor_msgs::msg::Image& image_msg,
             sensor_msgs::msg::CameraInfo& cam_model_msg);

bool convert(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg, rgbd::Image*& image);

} // namespace rgbd

#endif // RGBD_ROS_CONVERSIONS_H_
