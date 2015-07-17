#ifndef RGBD_TRANSPORT_VIEW_H_
#define RGBD_TRANSPORT_VIEW_H_

#include <rgbd/Image.h>

namespace rgbd {

class View {

public:

    View(const Image& image, int width);

    inline const int& getWidth() const
    {
        return width_;
    }

    inline const int& getHeight() const
    {
        return height_;
    }

    inline const cv::Vec3b& getColor(int x, int y) const
    {
        return image_.rgb_image_.at<cv::Vec3b>(y * rgb_factor_, x * rgb_factor_);
    }

    inline const float& getDepth(int x, int y) const
    {
        return image_.depth_image_.at<float>(y * depth_factor_, x * depth_factor_);
    }

    inline bool getPoint3D(int x, int y, geo::Vector3& p) const
    {
        float d = getDepth(x, y);
        p = rasterizer_.project2Dto3D(x, y) * d;
        return (d == d && d > 0);
    }

    inline bool getPoint3DSafe(int x, int y, geo::Vector3& p) const
    {
        if (x < 0 || y < 0 || x >= width_ || y >= height_)
            return false;

        float d = getDepth(x, y);
        p = rasterizer_.project2Dto3D(x, y) * d;
        return (d == d && d > 0);
    }

    inline const geo::DepthCamera& getRasterizer() const { return rasterizer_; }

protected:
    const Image& image_;
    int width_;
    int height_;

    float rgb_factor_;
    float depth_factor_;

    geo::DepthCamera rasterizer_;

};

}

#endif
