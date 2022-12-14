#ifndef RGBD_ROS_CONVERSIONS_H_
#define RGBD_ROS_CONVERSIONS_H_

#include <rgbd_msgs/RGBD.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "rgbd/image.h"

namespace cv { class Mat; }
namespace geo { class DepthCamera; }

namespace rgbd
{

/**
 * @brief Convert either a rgb or depth image, cv::Mat, to an image message
 * @param image image matrix
 * @param image_msg image message to fill
 * @return success
 */
bool convert(const cv::Mat& image, sensor_msgs::Image& image_msg);

/**
 * @brief Convert either a rgb or depth image to image and CameraInfo message. Also rectifies the image.
 * @param image rgb or depth image
 * @param cam_model Camera model
 * @param image_msg Image message
 * @param cam_model_msg CameraInfo message
 * @return
 */
bool convert(const cv::Mat& image, const geo::DepthCamera& cam_model, sensor_msgs::Image& image_msg, sensor_msgs::CameraInfo& cam_model_msg);

/**
 * @brief Convert rgbd message to an Image
 * @param msg pointer to const rgbd message
 * @param image raw pointer to an Image. In case it is a nullptr, a new instance will be created.
 * @return success
 */
bool convert(const rgbd_msgs::RGBDConstPtr& msg, rgbd::Image*& image);

}

#endif // RGBD_ROS_CONVERSIONS_H_
