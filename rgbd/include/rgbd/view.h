#ifndef RGBD_VIEW_H_
#define RGBD_VIEW_H_

#include <rgbd/image.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/DepthCamera.h>

namespace rgbd
{

class View
{

public:
    View(const Image& image, int width);

    [[nodiscard]]
    const int& getWidth() const
    {
        return width_;
    }

    [[nodiscard]]
    const int& getHeight() const
    {
        return height_;
    }

    [[nodiscard]]
    const cv::Vec3b& getColor(int x, int y) const
    {
        return image_.getRGBImage().at<cv::Vec3b>(static_cast<int>(static_cast<float>(y) * rgb_factor_),
                                                  static_cast<int>(static_cast<float>(x) * rgb_factor_));
    }

    [[nodiscard]]
    const float& getDepth(int x, int y) const
    {
        return image_.getDepthImage().at<float>(static_cast<int>(static_cast<float>(y) * depth_factor_),
                                                static_cast<int>(static_cast<float>(x) * depth_factor_));
    }

    bool getPoint3D(int x, int y, geo::Vector3& p) const
    {
        float const d = getDepth(x, y);
        p = rasterizer_.project2Dto3D(x, y) * d;
        return (d == d && d > 0);
    }

    bool getPoint3DSafe(int x, int y, geo::Vector3& p) const
    {
        if (x < 0 || y < 0 || x >= width_ || y >= height_)
            return false;

        float const d = getDepth(x, y);
        p = rasterizer_.project2Dto3D(x, y) * d;
        return (d == d && d > 0);
    }

    [[nodiscard]]
    const geo::DepthCamera& getRasterizer() const
    {
        return rasterizer_;
    }

protected:
    // View is intentionally a non-owning, non-assignable view over a longer-lived Image, mirroring
    // std::string_view/std::span.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    const Image& image_;
    int width_;
    int height_;

    float rgb_factor_;
    float depth_factor_;

    geo::DepthCamera rasterizer_;
};

} // namespace rgbd

#endif // RGBD_VIEW_H_
