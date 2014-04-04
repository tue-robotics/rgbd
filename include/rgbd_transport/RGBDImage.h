#ifndef RGBD_TRANSPORT_RGBDIMAGE_H_
#define RGBD_TRANSPORT_RGBDIMAGE_H_

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

class RGBDImage {

    friend class ROSModuleWrapper;

public:

    RGBDImage();

    virtual ~RGBDImage();

    void setRGBImage(const cv::Mat& img);

    void setDepthImage(const cv::Mat& img);

    void setFrameID(const std::string& frame_id);

    void setTimestamp(double stamp) { timestamp_ = stamp; }

    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model) { cam_model_ = cam_model; }

    const cv::Mat& getRGBImage() const;

    const cv::Mat& getDepthImage() const;

    const std::string& getFrameID() const;

    double getTimestamp() const { return timestamp_; }

    const image_geometry::PinholeCameraModel& getCameraModel() const { return cam_model_; }

protected:

    double timestamp_;
    std::string frame_id_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    image_geometry::PinholeCameraModel cam_model_;

};

#endif
