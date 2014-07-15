#include "rgbd/View.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

View::View(const Image& image, int width) :
    image_(image),
    width_(width)
{
    // Determine scaling between rgb and depth
    float aspect_ratio = (float) image.depth_image_.cols / image.depth_image_.rows; // 640 / 480
    height_ = width_ / aspect_ratio;

    // Factors
    rgb_factor_ = (float) image.rgb_image_.cols / width_;
    depth_factor_ = (float) image.depth_image_.cols / width_;

    // Unfortunately, the camera model interface of the geolib rasterizer is image size specific. Therefore,
    // create a new rasterizer based on the FOV of the given model, but scaled to the width and height given here.
    // ASSUMES: no optical translation in the given camera model
    // TODO: get rid of assumption
    float w_old = (image.rasterizer_.getOpticalCenterX() - 0.5) * 2;
    float h_old = (image.rasterizer_.getOpticalCenterY() - 0.5) * 2;

    rasterizer_.setFocalLengths(image.rasterizer_.getFocalLengthX() / w_old * width_,
                                image.rasterizer_.getFocalLengthY() / h_old * height_);
    rasterizer_.setOpticalCenter(width_ / 2 + 0.5, height_ / 2 + 0.5);
    rasterizer_.setOpticalTranslation(0, 0);
}

// ----------------------------------------------------------------------------------------

}
