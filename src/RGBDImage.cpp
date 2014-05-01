#include "rgbd_transport/RGBDImage.h"

namespace rgbd {

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

bool RGBDImage::getPoint3D(int x, int y, double& px, double& py, double& pz) const {
    float d = depth_image_.at<float>(y, x);
    if (d != d || d == 0) {
        return false;
    }

    cv::Point3d p = cam_model_.projectPixelTo3dRay(cv::Point2i(x, y)) * d;
    px = p.x;
    py = p.y;
    pz = p.z;

    return true;
}

}

