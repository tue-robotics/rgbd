#ifndef RGBD_TRANSPORT_RGBDIMAGE_H_
#define RGBD_TRANSPORT_RGBDIMAGE_H_

#include <opencv2/core/core.hpp>

class RGBDImage {

    friend class ROSModuleWrapper;

public:

    RGBDImage();

    virtual ~RGBDImage();

    void setRGBImage(const cv::Mat& img);

    void setDepthImage(const cv::Mat& img);

    void setFrameID(const std::string& frame_id);

    void setTimestamp(double stamp) { timestamp_ = stamp; }

    const cv::Mat& getRGBImage() const;

    const cv::Mat& getDepthImage() const;

    const std::string& getFrameID() const;

    double getTimestamp() const { return timestamp_; }

protected:

    double timestamp_;
    std::string frame_id_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

};

#endif
