#include "rgbd/view.h"
#include "rgbd/image.h"

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>
#else
#endif
#include <opencv2/core/mat.hpp>

namespace rgbd
{

// ----------------------------------------------------------------------------------------

View::View(const Image& image, int width) : image_(image), width_(width)
{
    const cv::Mat& rgb_image = image.getRGBImage();
    const cv::Mat& depth_image = image.getDepthImage();
    // Determine scaling between rgb and depth
    float const aspect_ratio = static_cast<float>(depth_image.cols) / static_cast<float>(depth_image.rows); // 640 / 480
    height_ = static_cast<int>(static_cast<float>(width_) / aspect_ratio);

    // Factors
    rgb_factor_ = static_cast<float>(rgb_image.cols) / static_cast<float>(width_);
    depth_factor_ = static_cast<float>(depth_image.cols) / static_cast<float>(width_);

    rasterizer_.initFromCamModel(image.getCameraModel());
}

} // namespace rgbd
