#include "rgbd_transport/RGBDImage.h"

namespace pein {

RGBDImage::RGBDImage() {

}

RGBDImage::~RGBDImage() {

}

void RGBDImage::setRGBImage(const cv::Mat& img) {
    rgb_image_ = img;
}

void RGBDImage::setDepthImage(const cv::Mat& img) {
    depth_image_ = img;
}

void RGBDImage::setFrameID(const std::string& frame_id) {
    frame_id_ = frame_id;
}

const cv::Mat& RGBDImage::getRGBImage() const {
    return rgb_image_;
}

const cv::Mat& RGBDImage::getDepthImage() const {
    return depth_image_;
}

const std::string& RGBDImage::getFrameID() const {
    return frame_id_;
}

}
