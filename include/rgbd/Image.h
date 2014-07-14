#ifndef RGBD_TRANSPORT_Image.h_
#define RGBD_TRANSPORT_Image.h_

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geolib/sensors/DepthCamera.h>

namespace tue
{
namespace serialization
{
    class InputArchive;
    class OutputArchive;
}
}

namespace rgbd {

class Image {

    friend class Server;
    friend class Client;
    friend class View;

public:

    Image();

    Image(const cv::Mat& rgb_image,
              const cv::Mat& depth_image,
              const image_geometry::PinholeCameraModel& cam_model,
              const std::string& frame_id,
              double timestamp);

    void setupRasterizer();

    inline const cv::Mat& getDepthImage() const { return depth_image_; }
    inline const cv::Mat& getRGBImage() const { return rgb_image_; }

    const image_geometry::PinholeCameraModel& getCameraModel() const { return cam_model_; }

    inline const std::string& getFrameId() const
    {
        return frame_id_;
    }

    inline const double& getTimestamp() const
    {
        return timestamp_;
    }

    friend void serialize(const Image& image, tue::serialization::OutputArchive& a);

    friend void deserialize(tue::serialization::InputArchive& a, Image& image);

protected:

    double timestamp_;
    std::string frame_id_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    image_geometry::PinholeCameraModel cam_model_;
    geo::DepthCamera rasterizer_;

};

}

#endif
