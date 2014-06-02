#include "rgbd_transport/RGBDImage.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

RGBDImage::RGBDImage() {
}

// ----------------------------------------------------------------------------------------

RGBDImage::~RGBDImage() {
}

// ----------------------------------------------------------------------------------------

void RGBDImage::setRGBImage(const cv::Mat& img) {
    rgb_image_ = img;
}

// ----------------------------------------------------------------------------------------

void RGBDImage::setDepthImage(const cv::Mat& img) {
    depth_image_ = img;

    // remove NaNs (zet to 0)
    for(int y = 0; y < depth_image_.rows; ++y) {
        for(int x = 0; x < depth_image_.cols; ++x) {
            float d = depth_image_.at<float>(y, x);
            if (d != d) {
                depth_image_.at<float>(y, x) = 0;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------

void RGBDImage::setCameraModel(const image_geometry::PinholeCameraModel& cam_model) {
    cam_model_ = cam_model;

    rasterizer_.setFocalLengths(cam_model_.fx(), cam_model_.fy());
    rasterizer_.setOpticalCenter(cam_model_.cx(), cam_model_.cy());
    rasterizer_.setOpticalTranslation(cam_model_.Tx(), cam_model_.Ty());
}

// ----------------------------------------------------------------------------------------

void RGBDImage::setFrameID(const std::string& frame_id) {
    frame_id_ = frame_id;
}

// ----------------------------------------------------------------------------------------

const cv::Mat& RGBDImage::getRGBImage() const {
    return rgb_image_;
}

// ----------------------------------------------------------------------------------------

const cv::Mat& RGBDImage::getDepthImage() const {
    return depth_image_;
}

// ----------------------------------------------------------------------------------------

const std::string& RGBDImage::getFrameID() const {
    return frame_id_;
}

// ----------------------------------------------------------------------------------------

bool RGBDImage::getPoint3D(int x, int y, double& px, double& py, double& pz) const {
    float d = depth_image_.at<float>(y, x);
    if (d != d || d == 0) {
        return false;
    }

    geo::Vector3 p = rasterizer_.project2Dto3D(x, y) * d;
    px = p.getX();
    py = p.getY();
    pz = p.getZ();

    return true;
}

// ----------------------------------------------------------------------------------------

bool RGBDImage::getPoint3D(int x, int y, geo::Vector3& p) const {
    double px, py, pz;
    if (!getPoint3D(x, y, px, py, pz)) {
        return false;
    }

    p = geo::Vector3(px, py, pz);

    return true;
}

// ----------------------------------------------------------------------------------------

bool RGBDImage::getPoint3DSafe(int x, int y, geo::Vector3& p) const {
    if (x < 0 || y < 0 || x >= getWidth() || y >= getHeight())
        return false;
    return getPoint3D(x, y, p);
}

// ----------------------------------------------------------------------------------------

void RGBDImage::getPCLPointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl) const
{
    double px,py,pz;
    for(int y = 0; y < depth_image_.rows; ++y) {
        for(int x = 0; x < depth_image_.cols; ++x) {
            getPoint3D(x,y,px,py,pz);

            pcl::PointXYZ p;
            p.x = px; p.y = py; p.z = pz;
            pcl.push_back(p);
        }
    }
}


}
