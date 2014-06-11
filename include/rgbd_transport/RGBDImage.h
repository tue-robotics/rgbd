#ifndef RGBD_TRANSPORT_RGBDIMAGE_H_
#define RGBD_TRANSPORT_RGBDIMAGE_H_

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geolib/sensors/DepthCamera.h>

#include <pcl/point_types.h>

namespace rgbd {

class RGBDImage {

    friend class ROSModuleWrapper;

public:

    RGBDImage();

    virtual ~RGBDImage();

    void setRGBImage(const cv::Mat& img);

    void setDepthImage(const cv::Mat& img);

    void setFrameID(const std::string& frame_id);

    void setTimestamp(double stamp) { timestamp_ = stamp; }

    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model);

    const cv::Mat& getRGBImage() const;

    const cv::Mat& getDepthImage() const;

    const std::string& getFrameID() const;

    double getTimestamp() const { return timestamp_; }

    int getWidth() const { return depth_image_.cols; }

    int getHeight() const { return depth_image_.rows; }

    const geo::DepthCamera& getRasterizer() const { return rasterizer_; }

    bool getPoint3D(int x, int y, double& px, double& py, double& pz) const;

    bool getPoint3D(int x, int y, geo::Vector3& p) const;

    bool getPoint3DSafe(int x, int y, geo::Vector3& p) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPCLPointCloud(int step = 1, int padding = 0, double max_range = 1e6) const;

    const image_geometry::PinholeCameraModel& getCameraModel() const { return cam_model_; }

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
