#ifndef RGBD_TRANSPORT_RGBDIMAGE_H_
#define RGBD_TRANSPORT_RGBDIMAGE_H_

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geolib/sensors/DepthCamera.h>

namespace rgbd {

class RGBDImage {

    friend class Server;
    friend class Client;

public:

    RGBDImage();

    RGBDImage(const cv::Mat& rgb_image,
              const cv::Mat& depth_image,
              const image_geometry::PinholeCameraModel& cam_model,
              const std::string& frame_id,
              double timestamp);

    inline const cv::Mat& getOriginalRGBImage() const {
        return rgb_image_;
    }

    inline const cv::Mat& getOriginalDepthImage() const {
        return depth_image_;
    }

    inline const int& getWidth() const
    {
        return width_;
    }

    inline const int& getHeight() const
    {
        return height_;
    }

    inline const std::string& getFrameId() const
    {
        return frame_id_;
    }

    inline const double& getTimestamp() const
    {
        return timestamp_;
    }

    inline const cv::Vec3b& getColor(int x, int y) const
    {
        return rgb_image_.at<cv::Vec3b>(y,x);
    }

    inline const float& getDepth(int x, int y) const
    {
        return depth_image_.at<float>(y/factor_,x/factor_);
    }

    inline bool getPoint3D(int x, int y, geo::Vector3& p) const
    {
        float d = getDepth(x,y);
        p = rasterizer_.project2Dto3D(x, y) * d;
        return (d == d);
    }

    cv::Point2i getIndexFromPoint3D(const geo::Vector3& p) const
    {
        return rasterizer_.project3Dto2D(p);
    }

    const image_geometry::PinholeCameraModel& getCameraModel() const { return cam_model_; }

    const geo::DepthCamera& getRasterizer() const { return rasterizer_; }

protected:

    double timestamp_;
    std::string frame_id_;

    int width_;
    int height_;
    float aspect_ratio_;
    int factor_; // for xbox kinect either 1 or 2 (640/640 or 1280/640)

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    image_geometry::PinholeCameraModel cam_model_;
    geo::DepthCamera rasterizer_;

    void updateRatio();

};

}

#endif
