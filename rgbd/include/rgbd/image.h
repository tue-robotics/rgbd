#ifndef RGBD_IMAGE_H_
#define RGBD_IMAGE_H_

#include "rgbd/types.h"

#include <cstdint>
#include <opencv2/core.hpp>
#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif
#include <rgbd_interfaces/msg/rgbd.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace tue::serialization
{
class InputArchive;
class OutputArchive;
} // namespace tue::serialization

namespace rgbd
{

enum class CameraModelType : int8_t
{
    CAMERA_MODEL_NONE = 0,
    CAMERA_MODEL_PINHOLE = 1
};

enum class RGBStorageType : int8_t
{
    RGB_STORAGE_NONE = 0,
    RGB_STORAGE_LOSSLESS = 1,
    RGB_STORAGE_JPG = 2
};

enum class DepthStorageType : int8_t
{
    DEPTH_STORAGE_NONE = 0,
    DEPTH_STORAGE_LOSSLESS = 1,
    DEPTH_STORAGE_PNG = 2
};

class Image
{
    friend class ClientSHM;

public:
    Image();

    Image(cv::Mat rgb_image,
          cv::Mat depth_image,
          const image_geometry::PinholeCameraModel& cam_model,
          std::string frame_id,
          double timestamp);

    [[nodiscard]]
    const cv::Mat& getDepthImage() const
    {
        return depth_image_;
    }
    [[nodiscard]]
    const cv::Mat& getRGBImage() const
    {
        return rgb_image_;
    }
    [[nodiscard]]
    const std::string& getFrameId() const
    {
        return frame_id_;
    }
    [[nodiscard]]
    double getTimestamp() const
    {
        return timestamp_;
    }
    [[nodiscard]]
    const image_geometry::PinholeCameraModel& getCameraModel() const
    {
        return cam_model_;
    }

    void setDepthImage(const cv::Mat& depth_image) { depth_image_ = depth_image; }
    void setRGBImage(const cv::Mat& rgb_image) { rgb_image_ = rgb_image; }
    void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    void setTimestamp(double timestamp) { timestamp_ = timestamp; }

    void setCameraInfo(sensor_msgs::msg::CameraInfo cam_info);
    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model);

    [[nodiscard]]
    Image clone() const;

    bool operator==(const rgbd::Image& other) const;
    bool operator!=(const rgbd::Image& other) const { return !(*this == other); }

    friend std::ostream& operator<<(std::ostream& out, const rgbd::Image& image);

    friend bool serialize(const Image& image,
                          tue::serialization::OutputArchive& a,
                          RGBStorageType rgb_type,
                          DepthStorageType depth_type);

    friend bool deserialize(tue::serialization::InputArchive& a, Image& image);

    friend bool convert(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg, rgbd::Image*& image);

protected:
    cv::Mat rgb_image_; // BGR format
    cv::Mat depth_image_;

    image_geometry::PinholeCameraModel cam_model_;

    std::string frame_id_;
    double timestamp_;
};

} // namespace rgbd

#endif // RGBD_IMAGE_H_
