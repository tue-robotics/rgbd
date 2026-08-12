#include "rgbd/serialization.h"
#include "rgbd/image.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <opencv2/imgcodecs.hpp>
#include <vector>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp> // IWYU pragma: keep
#else
#include <image_geometry/pinhole_camera_model.h> // IWYU pragma: keep
#endif

namespace rgbd
{

const static int SERIALIZATION_VERSION = 2;

// ----------------------------------------------------------------------------------------------------
//
//                                          SERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

bool serialize(const Image& image,
               tue::serialization::OutputArchive& a,
               RGBStorageType rgb_type,
               DepthStorageType depth_type)
{
    // - - - - - - - - - - - - - - - - GENERAL INFO - - - - - - - - - - - - - - - -

    a << SERIALIZATION_VERSION;

    a << image.getFrameId();
    a << image.getTimestamp();

    // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

    const image_geometry::PinholeCameraModel& cam_model = image.cam_model_;

    if (cam_model.initialized())
    {
        a << static_cast<int>(CameraModelType::CAMERA_MODEL_PINHOLE);
        a << cam_model.fx() << cam_model.fy();
        a << cam_model.cx() << cam_model.cy();
        a << cam_model.Tx() << cam_model.Ty();
        a << cam_model.fullResolution().width << cam_model.fullResolution().height;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("serialization"), "rgbd::serialize: cam_model not initialized");
        return false;
    }

    // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

    if (!image.rgb_image_.data)
        rgb_type = RGBStorageType::RGB_STORAGE_NONE;

    a << static_cast<int>(rgb_type);

    if (rgb_type == RGBStorageType::RGB_STORAGE_NONE) {}
    else if (rgb_type == RGBStorageType::RGB_STORAGE_LOSSLESS)
    {
        a << image.rgb_image_.cols;
        a << image.rgb_image_.rows;

        const int size = image.rgb_image_.rows * image.rgb_image_.cols * 3;
        a.write(reinterpret_cast<const char*>(image.rgb_image_.data), size);
    }
    else if (rgb_type == RGBStorageType::RGB_STORAGE_JPG)
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = cv::IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        std::vector<unsigned char> rgb_data;

        // Compress image
        if (!cv::imencode(".jpg", image.rgb_image_, rgb_data, rgb_params))
        {
            RCLCPP_ERROR(rclcpp::get_logger("serialization"), "RGB image compression failed");
            return false;
        }

        a << static_cast<int>(rgb_data.size());
        a.write(reinterpret_cast<const char*>(rgb_data.data()), rgb_data.size());
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("serialization"), "Unsupported RGB STORAGE TYPE: %d", static_cast<int>(rgb_type));
        return false;
    }

    // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

    if (!image.depth_image_.data)
        depth_type = DepthStorageType::DEPTH_STORAGE_NONE;

    a << static_cast<int>(depth_type);

    if (depth_type == DepthStorageType::DEPTH_STORAGE_NONE) {}
    else if (depth_type == DepthStorageType::DEPTH_STORAGE_LOSSLESS)
    {
        a << image.depth_image_.cols;
        a << image.depth_image_.rows;

        const int size = image.depth_image_.rows * image.depth_image_.cols * 4;
        a.write(reinterpret_cast<const char*>(image.depth_image_.data), size);
    }
    else if (depth_type == DepthStorageType::DEPTH_STORAGE_PNG)
    {
        const float depth_z0 = 100; // config_.depth_quantization;
        const float depth_max = 10; // config_.depth_max;

        const float depth_quant_a = depth_z0 * (depth_z0 + 1.0f);
        const float depth_quant_b = 1.0f - (depth_quant_a / depth_max);

        a << depth_quant_a << depth_quant_b;

        const cv::Mat& depth_image = image.depth_image_;
        cv::Mat inv_depth_img(depth_image.size(), CV_16UC1);

        // Matrix iterators
        cv::MatConstIterator_<float> it_depth_img = depth_image.begin<float>();
        const cv::MatConstIterator_<float> it_depth_img_end = depth_image.end<float>();
        cv::MatIterator_<uint16_t> it_inv_depth_img = inv_depth_img.begin<uint16_t>();
        const cv::MatIterator_<uint16_t> it_inv_depth_img_end = inv_depth_img.end<uint16_t>();

        // Quantization
        for (; (it_depth_img != it_depth_img_end) && (it_inv_depth_img != it_inv_depth_img_end);
             ++it_depth_img, ++it_inv_depth_img)
        {
            // check for NaN & max depth
            if (*it_depth_img < depth_max)
            {
                *it_inv_depth_img = static_cast<uint16_t>((depth_quant_a / *it_depth_img) + depth_quant_b);
            }
            else
            {
                *it_inv_depth_img = 0;
            }
        }

        // Compression settings
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = cv::IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        std::vector<unsigned char> depth_data;

        if (!cv::imencode(".png", inv_depth_img, depth_data, params))
        {
            RCLCPP_ERROR(rclcpp::get_logger("serialization"), "Depth image compression failed");
            return false;
        }

        a << static_cast<int>(depth_data.size());
        a.write(reinterpret_cast<const char*>(depth_data.data()), depth_data.size());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("serialization"), "Unsupported DEPTH_STORAGE_TYPE");
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------
//
//                                         DESERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::serialization::InputArchive& a, Image& image)
{
    // - - - - - - - - - - - - - - - - GENERAL INFO - - - - - - - - - - - - - - - -

    int version = 0;
    a >> version;

    a >> image.frame_id_;
    a >> image.timestamp_;

    // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

    int cam_type = 0;
    a >> cam_type;

    if (cam_type == static_cast<int>(CameraModelType::CAMERA_MODEL_NONE)) {}
    else if (cam_type == static_cast<int>(CameraModelType::CAMERA_MODEL_PINHOLE))
    {
        double fx = NAN;
        double fy = NAN;
        double cx = NAN;
        double cy = NAN;
        double tx = NAN;
        double ty = NAN;
        int width = 0;
        int height = 0;
        a >> fx >> fy;
        a >> cx >> cy;
        a >> tx >> ty;
        if (version >= 2)
            a >> width >> height;

        sensor_msgs::msg::CameraInfo cam_info_msg;

        cam_info_msg.d.resize(5, 0.0);
        cam_info_msg.k.fill(0.0);
        cam_info_msg.k[0] = fx; // fx
        cam_info_msg.k[2] = cx; // cx
        cam_info_msg.k[4] = fy; // fy
        cam_info_msg.k[5] = cy; // cy
        cam_info_msg.k[8] = 1.0;

        cam_info_msg.r.fill(0.0);
        cam_info_msg.r[0] = 1.0;
        cam_info_msg.r[4] = 1.0;
        cam_info_msg.r[8] = 1.0;

        cam_info_msg.p.fill(0.0);
        cam_info_msg.p[0] = fx; // fx
        cam_info_msg.p[2] = cx; // cx
        cam_info_msg.p[3] = tx; // Tx
        cam_info_msg.p[5] = fy; // fy
        cam_info_msg.p[6] = cy; // cy
        cam_info_msg.p[7] = ty; // Ty
        cam_info_msg.p[10] = 1.0;

        cam_info_msg.distortion_model = "plumb_bob";
        if (version >= 2)
        {
            cam_info_msg.width = width;
            cam_info_msg.height = height;
        }
        image.cam_model_.fromCameraInfo(cam_info_msg);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("serialization"), "rgbd::deserialize: Unsupported camera model: %d", cam_type);
        return false;
    }

    // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

    int rgb_type = 0;
    a >> rgb_type;

    if (rgb_type == static_cast<int>(RGBStorageType::RGB_STORAGE_NONE)) {}
    else if (rgb_type == static_cast<int>(RGBStorageType::RGB_STORAGE_LOSSLESS))
    {
        int width = 0;
        int height = 0;
        a >> width;
        a >> height;

        const int size = width * height * 3;
        image.rgb_image_ = cv::Mat(height, width, CV_8UC3);
        for (int i = 0; i < size; ++i)
            a >> image.rgb_image_.data[i];
    }
    else if (rgb_type == static_cast<int>(RGBStorageType::RGB_STORAGE_JPG))
    {
        int rgb_size = 0;
        a >> rgb_size;

        std::vector<unsigned char> rgb_data(rgb_size);
        for (int i = 0; i < rgb_size; ++i)
            a >> rgb_data[i];

        image.rgb_image_ = cv::imdecode(rgb_data, cv::IMREAD_UNCHANGED);
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("serialization"), "rgbd::deserialize: Unsupported rgb storage format: %d", rgb_type);
        return false;
    }

    // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

    int depth_type = 0;
    a >> depth_type;

    if (depth_type == static_cast<int>(DepthStorageType::DEPTH_STORAGE_NONE)) {}
    else if (depth_type == static_cast<int>(DepthStorageType::DEPTH_STORAGE_LOSSLESS))
    {
        int width = 0;
        int height = 0;
        a >> width;
        a >> height;

        const int size = width * height * 4;
        image.depth_image_ = cv::Mat(height, width, CV_32FC1);
        for (int i = 0; i < size; ++i)
            a >> image.depth_image_.data[i];
    }
    else if (depth_type == static_cast<int>(DepthStorageType::DEPTH_STORAGE_PNG))
    {
        float depth_quant_a = NAN;
        float depth_quant_b = NAN;
        a >> depth_quant_a >> depth_quant_b;

        int depth_size = 0;
        a >> depth_size;

        std::vector<unsigned char> depth_data(depth_size);
        for (int i = 0; i < depth_size; ++i)
            a >> depth_data[i];

        cv::Mat decompressed = cv::imdecode(depth_data, cv::IMREAD_UNCHANGED);
        cv::Mat& depth_image = image.depth_image_;
        depth_image = cv::Mat(decompressed.size(), CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> it_depth_img = depth_image.begin<float>();
        const cv::MatIterator_<float> it_depth_img_end = depth_image.end<float>();
        cv::MatConstIterator_<uint16_t> it_inv_depth_img = decompressed.begin<uint16_t>();
        const cv::MatConstIterator_<uint16_t> it_inv_depth_img_end = decompressed.end<uint16_t>();

        for (; (it_depth_img != it_depth_img_end) && (it_inv_depth_img != it_inv_depth_img_end);
             ++it_depth_img, ++it_inv_depth_img)
        {
            // check for NaN & max depth
            if (*it_inv_depth_img)
            {
                *it_depth_img = depth_quant_a / (static_cast<float>(*it_inv_depth_img) - depth_quant_b);
            }
            else
            {
                *it_depth_img = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("serialization"), "rgbd::deserialize: Unsupported depth storage format: %d", depth_type);
        return false;
    }

    return true;
}

} // namespace rgbd
