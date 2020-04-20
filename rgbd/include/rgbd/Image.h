#ifndef RGBD_TRANSPORT_IMAGE_H_
#define RGBD_TRANSPORT_IMAGE_H_

#include "rgbd/types.h"

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geolib/sensors/DepthCamera.h>
#include <rgbd_msgs/RGBD.h>

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

    friend class Server;
    friend class ServerRGBD;
    friend class ServerShmOnly;
    friend class Client;
    friend class ClientRGBD;
    friend class ClientROS;
    friend class SharedMemClient;
    friend class View;

public:

    Image()
    {
    }

    Image(const cv::Mat& rgb_image,
          const cv::Mat& depth_image,
          const image_geometry::PinholeCameraModel& cam_model,
          const std::string& frame_id,
          double timestamp) :
    rgb_image_(rgb_image),
    depth_image_(depth_image),
    cam_model_(cam_model),
    frame_id_(frame_id),
    timestamp_(timestamp)
    {
    }

    inline const cv::Mat& getDepthImage() const { return depth_image_; }
    inline const cv::Mat& getRGBImage() const { return rgb_image_; }

    inline const std::string& getFrameId() const
    {
        return frame_id_;
    }

    inline const double& getTimestamp() const
    {
        return timestamp_;
    }

    inline const image_geometry::PinholeCameraModel& getCameraModel() const
    {
        return cam_model_;
    }

    inline void setDepthImage(const cv::Mat& depth_image) { depth_image_ = depth_image; }
    inline void setRGBImage(const cv::Mat& rgb_image) { rgb_image_ = rgb_image; }
    inline void setFrameId(const std::string frame_id) { frame_id_ = frame_id; }
    inline void setTimestamp(const double timestamp) { timestamp_ = timestamp; }

    Image clone() const
    {
        return Image(rgb_image_.clone(), depth_image_.clone(), cam_model_, frame_id_, timestamp_);
    }

    friend bool serialize(const Image& image, tue::serialization::OutputArchive& a,
                          RGBStorageType rgb_type,
                          DepthStorageType depth_type);

    friend bool deserialize(tue::serialization::InputArchive& a, Image& image);

    friend bool convert(rgbd_msgs::RGBDConstPtr msg, rgbd::Image* image);

protected:

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    image_geometry::PinholeCameraModel cam_model_;

    std::string frame_id_;
    double timestamp_;

};

}

#endif
