#include "rgbd/image.h"

#include <cstdlib>
#include <iomanip>
#include <ios>
#include <opencv2/core.hpp>
#include <opencv2/core/check.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <ostream>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/camera_info__traits.hpp>
#include <string>
#include <utility>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp> // IWYU pragma: keep
#else
#include <image_geometry/pinhole_camera_model.h> // IWYU pragma: keep
#endif

namespace rgbd
{

Image::Image() : timestamp_(0) {}

Image::Image(cv::Mat rgb_image,
             cv::Mat depth_image,
             const image_geometry::PinholeCameraModel& cam_model,
             std::string frame_id,
             double timestamp) :
    rgb_image_(std::move(rgb_image)), depth_image_(std::move(depth_image)), frame_id_(std::move(frame_id)),
    timestamp_(timestamp)
{
    setCameraModel(cam_model);
}

void Image::setCameraInfo(sensor_msgs::msg::CameraInfo cam_info)
{
    cam_info.header.frame_id.clear();
    cam_info.header.stamp = rclcpp::Time(0, 0);
    cam_model_.fromCameraInfo(cam_info);
}

void Image::setCameraModel(const image_geometry::PinholeCameraModel& cam_model)
{
    sensor_msgs::msg::CameraInfo cam_info = cam_model.cameraInfo();
    cam_info.header.frame_id.clear();
    cam_info.header.stamp = rclcpp::Time(0, 0);
    cam_model_.fromCameraInfo(cam_info);
}

Image Image::clone() const
{
    rgbd::Image image;
    image.rgb_image_ = rgb_image_.clone();
    image.depth_image_ = depth_image_.clone();
    image.frame_id_ = frame_id_;
    image.timestamp_ = timestamp_;
    if (cam_model_.initialized())
    {
        image.setCameraModel(cam_model_);
    }

    return image;
}

bool Image::operator==(const rgbd::Image& other) const
{
    if (getTimestamp() > 0 && std::abs<double>(getTimestamp() - other.getTimestamp()) > 1e-9)
        return false;
    if (!getFrameId().empty() && getFrameId() != other.getFrameId())
        return false;
    if (getCameraModel().cameraInfo() != other.getCameraModel().cameraInfo())
        return false;

    const cv::Mat& this_depth = getDepthImage();
    const cv::Mat& other_depth = other.getDepthImage();
    if (this_depth.data != other_depth.data)
    {
        cv::Mat dst;
        cv::bitwise_xor(this_depth, other_depth, dst);
        if (cv::countNonZero(dst) != 0)
            return false;
    }

    const cv::Mat& this_rgb = getRGBImage();
    const cv::Mat& other_rgb = other.getRGBImage();
    if (this_rgb.data != other_rgb.data)
    {
        cv::Mat dst;
        cv::Mat dst2;
        cv::bitwise_xor(this_rgb, other_rgb, dst);
        cv::transform(dst, dst2, cv::Matx<int, 1, 3>(1, 1, 1));
        if (cv::countNonZero(dst2) != 0)
            return false;
    }

    return true;
}

std::ostream& operator<<(std::ostream& out, const rgbd::Image& image)
{
    std::streamsize const ss = out.precision();
    out << "Depth: " << image.depth_image_.size << "@(" << cv::typeToString(image.depth_image_.type()) << ")" << '\n'
        << "color: " << image.rgb_image_.size << "@(" << cv::typeToString(image.rgb_image_.type()) << ")" << '\n'
        << "frame_id: " << image.frame_id_ << '\n'
        << "timestamp: " << std::setprecision(32) << image.timestamp_ << std::setprecision(static_cast<int>(ss)) << '\n'
        << "camera model: " << '\n'
        << sensor_msgs::msg::to_yaml(image.cam_model_.cameraInfo());
    return out;
}

} // namespace rgbd
