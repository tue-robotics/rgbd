#include "rgbd/view.h"

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

    // ASSUMPTION: here we assume that the camera model given in the image is based
    // on the depth image, not the rgb image
    float w_depth = image.depth_image_.cols;
    float h_depth = image.depth_image_.rows;

    rasterizer_.setFocalLengths(image.cam_model_.fx() / w_depth * width_,
                                image.cam_model_.fy() / h_depth * height_);
    rasterizer_.setOpticalCenter(image.cam_model_.cx() / w_depth * width_,
                                 image.cam_model_.cy() / h_depth * height_);
    rasterizer_.setOpticalTranslation(0, 0);
}

// ----------------------------------------------------------------------------------------

}
