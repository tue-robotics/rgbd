#include "rgbd/view.h"

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/mat.hpp>

namespace rgbd {

// ----------------------------------------------------------------------------------------

View::View(const Image& image, int width) :
    image_(image), width_(width)
{
    const cv::Mat& rgb_image = image.getRGBImage();
    const cv::Mat& depth_image = image.getDepthImage();
    // Determine scaling between rgb and depth
    float aspect_ratio = static_cast<float>(depth_image.cols) / static_cast<float>(depth_image.rows); // 640 / 480
    height_ = static_cast<int>(width_ / aspect_ratio);

    // Factors
    rgb_factor_ = static_cast<float>(rgb_image.cols) / static_cast<float>(width_);
    depth_factor_ = static_cast<float>(depth_image.cols) / static_cast<float>(width_);

    // ASSUMPTION: here we assume that the camera model given in the image is based
    // on the depth image, not the rgb image
    float w_depth = depth_image.cols;
    float h_depth = depth_image.rows;

    const image_geometry::PinholeCameraModel& cam_model = image.getCameraModel();

    rasterizer_.setFocalLengths(cam_model.fx() / static_cast<double>(w_depth) * width_,
                                cam_model.fy() / static_cast<double>(h_depth) * height_);
    rasterizer_.setOpticalCenter(cam_model.cx() / static_cast<double>(w_depth) * width_,
                                 cam_model.cy() / static_cast<double>(h_depth) * height_);
    rasterizer_.setOpticalTranslation(0, 0);
}

}
