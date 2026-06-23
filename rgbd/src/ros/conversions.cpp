#include "rgbd/ros/conversions.h"

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif


#include <geolib/ros/msg_conversions.h>
#include <geolib/sensors/DepthCamera.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>

#include <tue/serialization/conversions.h>

#include <sstream>
#include <vector>

#include "rgbd/serialization.h"

namespace rgbd
{

bool convert(const cv::Mat& image, sensor_msgs::msg::Image& image_msg)
{
    cv_bridge::CvImage image_cv_bridge;

    if (image.type() == CV_32FC1)
        image_cv_bridge.encoding = "32FC1";
    else if (image.type() == CV_8UC3)
        image_cv_bridge.encoding = "bgr8";
    else
        return false;

    image_cv_bridge.image = image;
    image_cv_bridge.toImageMsg(image_msg);

    return true;
}

bool convert(const cv::Mat& image,
             const geo::DepthCamera& cam_model,
             sensor_msgs::msg::Image& image_msg,
             sensor_msgs::msg::CameraInfo& cam_model_msg)
{
    geo::convert(cam_model, cam_model_msg);
    int width = static_cast<int>(cam_model_msg.width);
    int height = static_cast<int>(cam_model_msg.height);

    cv_bridge::CvImage image_cv_bridge;

    cv::Mat img_rect;
    if (image.type() == CV_32FC1)
    {
        image_cv_bridge.encoding = "32FC1";
        img_rect = cv::Mat(height, width, CV_32FC1, cv::Scalar(0.));
    }
    else if (image.type() == CV_8UC3)
    {
        image_cv_bridge.encoding = "bgr8";
        img_rect = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    else
        return false;

    cv::Rect crop_rect(0, 0, std::min(img_rect.cols, image.cols), std::min(img_rect.rows, image.rows));
    image(crop_rect).copyTo(img_rect.rowRange(0, crop_rect.height).colRange(0, crop_rect.width));

    image_cv_bridge.image = img_rect;
    image_cv_bridge.toImageMsg(image_msg);

    return true;
}

bool convert(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg, rgbd::Image*& image)
{
    if (!image)
        image = new rgbd::Image;

    if (msg->version == 1)
    {
        std::vector<uint8_t> rgb_data(msg->rgb.begin(), msg->rgb.end());
        image->rgb_image_ = cv::imdecode(rgb_data, cv::IMREAD_UNCHANGED);

        float depthQuantA = static_cast<float>(msg->params[0]);
        float depthQuantB = static_cast<float>(msg->params[1]);

        std::vector<uint8_t> depth_data(msg->depth.begin(), msg->depth.end());
        cv::Mat decompressed = cv::imdecode(depth_data, cv::IMREAD_UNCHANGED);
        image->depth_image_ = cv::Mat(decompressed.size(), CV_32FC1);

        cv::MatIterator_<float> itDepthImg = image->depth_image_.begin<float>(), itDepthImg_end = image->depth_image_.end<float>();
        cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(), itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
            if (*itInvDepthImg)
                *itDepthImg = depthQuantA / (static_cast<float>(*itInvDepthImg) - depthQuantB);
            else
                *itDepthImg = std::numeric_limits<float>::quiet_NaN();
        }

        sensor_msgs::msg::CameraInfo cam_info_msg;

        cam_info_msg.d.resize(5, 0.0);
        cam_info_msg.k.fill(0.0);
        cam_info_msg.k[0] = msg->cam_info[0];
        cam_info_msg.k[2] = msg->cam_info[2];
        cam_info_msg.k[4] = msg->cam_info[1];
        cam_info_msg.k[5] = msg->cam_info[3];
        cam_info_msg.k[8] = 1.0;

        cam_info_msg.r.fill(0.0);
        cam_info_msg.r[0] = 1.0;
        cam_info_msg.r[4] = 1.0;
        cam_info_msg.r[8] = 1.0;

        cam_info_msg.p.fill(0.0);
        cam_info_msg.p[0] = msg->cam_info[0];
        cam_info_msg.p[2] = msg->cam_info[2];
        cam_info_msg.p[3] = msg->cam_info[4];
        cam_info_msg.p[5] = msg->cam_info[1];
        cam_info_msg.p[6] = msg->cam_info[3];
        cam_info_msg.p[7] = msg->cam_info[5];
        cam_info_msg.p[10] = 1.0;

        cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        cam_info_msg.width = static_cast<unsigned int>(image->rgb_image_.cols);
        cam_info_msg.height = static_cast<unsigned int>(image->rgb_image_.rows);
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info_msg);

        image->cam_model_ = cam_model;
        image->timestamp_ = rclcpp::Time(msg->header.stamp).seconds();
        image->frame_id_ = msg->header.frame_id;
        return true;
    }
    if (msg->version == 2)
    {
        std::stringstream stream;
        tue::serialization::convert(msg->rgb, stream);
        tue::serialization::InputArchive a(stream);
        return rgbd::deserialize(a, *image);
    }
    if (msg->version == 3)
    {
        std::stringstream compressed;
        std::stringstream decompressed;
        tue::serialization::convert(msg->rgb, compressed);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(compressed);
        boost::iostreams::copy(in, decompressed);
        tue::serialization::InputArchive a(decompressed);
        return rgbd::deserialize(a, *image);
    }
    if (msg->version == 4)
    {
        std::stringstream compressed;
        std::stringstream decompressed;
        tue::serialization::convert(msg->rgb, compressed);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::zstd_decompressor());
        in.push(compressed);
        boost::iostreams::copy(in, decompressed);
        tue::serialization::InputArchive a(decompressed);
        return rgbd::deserialize(a, *image);
    }

    RCLCPP_ERROR(rclcpp::get_logger("conversions"), "convert: version '%d' not supported", msg->version);
    return false;
}

} // namespace rgbd
