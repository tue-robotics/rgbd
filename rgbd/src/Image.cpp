#include "rgbd/Image.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

Image::Image()
{
}

// ----------------------------------------------------------------------------------------

Image::Image(const cv::Mat& rgb_image,
             const cv::Mat& depth_image,
             const image_geometry::PinholeCameraModel& cam_model,
             const std::string& frame_id,
             double timestamp) :
    rgb_image_(rgb_image),
    depth_image_(depth_image),
    cam_model_(cam_model),
    frame_id_(frame_id),
    timestamp_(timestamp)
{
    setupRasterizer();
}

// ----------------------------------------------------------------------------------------

Image::Image(const cv::Mat& rgb_image,
             const cv::Mat& depth_image,
             const geo::DepthCamera& cam_model,
             const std::string& frame_id,
             double timestamp) :
    rgb_image_(rgb_image),
    depth_image_(depth_image),
    rasterizer_(cam_model),
    frame_id_(frame_id),
    timestamp_(timestamp)
{
}

// ----------------------------------------------------------------------------------------

void Image::setupRasterizer()
{
    if (cam_model_.initialized())
    {
        // Set the rasterizer
        rasterizer_.setFocalLengths(cam_model_.fx(), cam_model_.fy());
        rasterizer_.setOpticalCenter(cam_model_.cx(), cam_model_.cy());
        rasterizer_.setOpticalTranslation(cam_model_.Tx(), cam_model_.Ty());
    }
}

// ----------------------------------------------------------------------------------------

}
