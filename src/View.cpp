#include "rgbd/View.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

View::View(const Image& image, int width) :
    image_(image),
    width_(width),
    rasterizer_(image.rasterizer_)
{
    // Determine scaling between rgb and depth
    float aspect_ratio = (float) image.depth_image_.cols / image.depth_image_.rows; // 640 / 480
    height_ = width_ / aspect_ratio;

    // Factors
    rgb_factor_ = (float) image.rgb_image_.cols / width_;
    depth_factor_ = (float) image.depth_image_.cols / width_;
}

// ----------------------------------------------------------------------------------------

}
