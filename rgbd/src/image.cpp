#include "rgbd/image.h"

#include <opencv2/core/check.hpp>

#include <sensor_msgs/CameraInfo.h>


namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

Image::Image() : timestamp_(0)
{
}

// ----------------------------------------------------------------------------------------------------

Image::Image(const cv::Mat& rgb_image,
             const cv::Mat& depth_image,
             const image_geometry::PinholeCameraModel& cam_model,
             const std::string& frame_id,
             double timestamp) :
    rgb_image_(rgb_image),
    depth_image_(depth_image),
    frame_id_(frame_id),
    timestamp_(timestamp)
{
    setCameraModel(cam_model);
}

// ----------------------------------------------------------------------------------------------------

void Image::setCameraInfo(sensor_msgs::CameraInfo cam_info)
{
    cam_info.header.frame_id.clear();
    cam_info.header.seq = 0;
    cam_info.header.stamp.fromSec(0);
    cam_model_.fromCameraInfo(cam_info);
}


// ----------------------------------------------------------------------------------------------------

void Image::setCameraModel(const image_geometry::PinholeCameraModel& cam_model)
{
    sensor_msgs::CameraInfo cam_info = cam_model.cameraInfo();
    cam_info.header.frame_id.clear();
    cam_info.header.seq = 0;
    cam_info.header.stamp.fromSec(0);
    cam_model_.fromCameraInfo(cam_info);
}

// ----------------------------------------------------------------------------------------------------

Image Image::clone() const
{
    rgbd::Image image;
    image.rgb_image_ = rgb_image_.clone();
    image.depth_image_ = depth_image_.clone();
    image.frame_id_ = frame_id_;
    image.timestamp_ = timestamp_;
    if (cam_model_.initialized())
        image.setCameraModel(cam_model_);

    return image;
}

// ----------------------------------------------------------------------------------------------------

bool Image::operator==(const rgbd::Image& other) const
{
    if (getTimestamp() > 0 && getTimestamp() != other.getTimestamp())
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
        cv::Mat dst, dst2;
        cv::bitwise_xor(this_rgb, other_rgb, dst);
        cv::transform(dst, dst2, cv::Matx<int, 1, 3>(1,1,1));
        if (cv::countNonZero(dst2) != 0)
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream& out, const rgbd::Image& image)
{
    out << "Depth: " << image.depth_image_.size << "@(" << cv::typeToString(image.depth_image_.type()) << ")" << std::endl
        << "color: " << image.rgb_image_.size << "@(" << cv::typeToString(image.rgb_image_.type()) << ")" << std::endl
        << "frame_id: " << image.frame_id_ << std::endl
        << "timestamp: " << image.timestamp_ << std::endl
        << "camera model: " << std::endl << image.cam_model_.cameraInfo();
    return out;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace rgbd
