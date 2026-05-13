#ifndef RGBD_IMAGE_H_
#define RGBD_IMAGE_H_

#include "rgbd/types.h"

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <rgbd_interfaces/msg/rgbd.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace tue
{
namespace serialization
{
class InputArchive;
class OutputArchive;
}
}

namespace rgbd {

enum CameraModelType
{
    CAMERA_MODEL_NONE = 0,
    CAMERA_MODEL_PINHOLE = 1
};

enum RGBStorageType
{
    RGB_STORAGE_NONE = 0,
    RGB_STORAGE_LOSSLESS = 1,
    RGB_STORAGE_JPG = 2
};

enum DepthStorageType
{
    DEPTH_STORAGE_NONE = 0,
    DEPTH_STORAGE_LOSSLESS = 1,
    DEPTH_STORAGE_PNG = 2
};

class Image {
    friend class ClientSHM;

public:
    Image();

    Image(const cv::Mat& rgb_image,
          const cv::Mat& depth_image,
          const image_geometry::PinholeCameraModel& cam_model,
          const std::string& frame_id,
          double timestamp);

    inline const cv::Mat& getDepthImage() const { return depth_image_; }
    inline const cv::Mat& getRGBImage() const { return rgb_image_; }
    inline const std::string& getFrameId() const { return frame_id_; }
    inline double getTimestamp() const { return timestamp_; }
    inline const image_geometry::PinholeCameraModel& getCameraModel() const { return cam_model_; }

    inline void setDepthImage(const cv::Mat& depth_image) { depth_image_ = depth_image; }
    inline void setRGBImage(const cv::Mat& rgb_image) { rgb_image_ = rgb_image; }
    inline void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    inline void setTimestamp(double timestamp) { timestamp_ = timestamp; }

    void setCameraInfo(sensor_msgs::msg::CameraInfo cam_info);
    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model);

    Image clone() const;

    bool operator==(const rgbd::Image& other) const;
    inline bool operator!=(const rgbd::Image& other) const { return !(*this == other); }

    friend std::ostream& operator<< (std::ostream& out, const rgbd::Image& image);

    friend bool serialize(const Image& image, tue::serialization::OutputArchive& a,
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

}  // namespace rgbd

#endif // RGBD_IMAGE_H_
