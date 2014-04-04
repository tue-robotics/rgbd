#ifndef PEIN_CORE_RGBD_IMAGE_H_
#define PEIN_CORE_RGBD_IMAGE_H_

#include <opencv2/core/core.hpp>

namespace pein {

class RGBDImage {

    friend class ROSModuleWrapper;

public:

    RGBDImage();

    virtual ~RGBDImage();

    void setRGBImage(const cv::Mat& img);

    void setDepthImage(const cv::Mat& img);

    void setFrameID(const std::string& frame_id);

    const cv::Mat& getRGBImage() const;

    const cv::Mat& getDepthImage() const;

    const std::string& getFrameID() const;

protected:

    double timestamp_;

    std::string frame_id_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

};

}

#endif
