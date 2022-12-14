#include "rgbd/ros/conversions.h"

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zstd.hpp>

#include <cv_bridge/cv_bridge.h>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/ros/msg_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ros/console.h>

#include <sensor_msgs/distortion_models.h>

#include <tue/serialization/conversions.h>

#include <sstream>

#include "rgbd/serialization.h"


namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

bool convert(const cv::Mat& image, sensor_msgs::Image& image_msg)
{
    cv_bridge::CvImage image_cv_bridge;

    if (image.type() == CV_32FC1) // Depth image
        image_cv_bridge.encoding = "32FC1";
    else if (image.type() == CV_8UC3) // RGB image;
        image_cv_bridge.encoding = "bgr8";
    else
        return false;

    image_cv_bridge.image = image;
    image_cv_bridge.toImageMsg(image_msg);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool convert(const cv::Mat& image, 
             const geo::DepthCamera& cam_model, 
             sensor_msgs::Image& image_msg, 
             sensor_msgs::CameraInfo& cam_model_msg)
{
    geo::convert(cam_model, cam_model_msg);
    int width = static_cast<int>(cam_model_msg.width);
    int height = static_cast<int>(cam_model_msg.height);

    cv_bridge::CvImage image_cv_bridge;

    cv::Mat img_rect;
    if (image.type() == CV_32FC1) // Depth image
    {
        image_cv_bridge.encoding = "32FC1";
        img_rect = cv::Mat(height, width, CV_32FC1, cv::Scalar(0.));
    }
    else if (image.type() == CV_8UC3) // RGB image
    {
        image_cv_bridge.encoding = "bgr8";
        img_rect = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    else
        return false;

    // Create a ROI for the part of the image that we need to copy
    cv::Rect crop_rect(0, 0, std::min(img_rect.cols, image.cols), std::min(img_rect.rows, image.rows));
    image(crop_rect).copyTo(img_rect.rowRange(0, crop_rect.height).colRange(0, crop_rect.width));

    image_cv_bridge.image = img_rect;
    image_cv_bridge.toImageMsg(image_msg);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool convert(const rgbd_msgs::RGBDConstPtr& msg, rgbd::Image*& image)
{
    if (!image)
        image = new rgbd::Image;

    if (msg->version == 1)
    {
        // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

        image->rgb_image_ = cv::imdecode(cv::Mat(msg->rgb), cv::IMREAD_UNCHANGED);

        // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

        float depthQuantA = static_cast<float>(msg->params[0]);
        float depthQuantB = static_cast<float>(msg->params[1]);

        cv::Mat decompressed = cv::imdecode(msg->depth, cv::IMREAD_UNCHANGED);
        image->depth_image_ = cv::Mat(decompressed.size(), CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> itDepthImg = image->depth_image_.begin<float>(),
                itDepthImg_end = image->depth_image_.end<float>();
        cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
            // check for NaN & max depth
            if (*itInvDepthImg)
            {
                *itDepthImg = depthQuantA / (static_cast<float>(*itInvDepthImg) - depthQuantB);
            } else{
                *itDepthImg = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.D.resize(5, 0.0);
        cam_info_msg.K.fill(0.0);
        cam_info_msg.K[0] = msg->cam_info[0];  // fx
        cam_info_msg.K[2] = msg->cam_info[2];  // cx
        cam_info_msg.K[4] = msg->cam_info[1];  // fy
        cam_info_msg.K[5] = msg->cam_info[3];  // cy
        cam_info_msg.K[8] = 1.0;

        cam_info_msg.R.fill(0.0);
        cam_info_msg.R[0] = 1.0;
        cam_info_msg.R[4] = 1.0;
        cam_info_msg.R[8] = 1.0;

        cam_info_msg.P.fill(0.0);
        cam_info_msg.P[0] = msg->cam_info[0];  // fx
        cam_info_msg.P[2] = msg->cam_info[2];  // cx
        cam_info_msg.P[3] = msg->cam_info[4];  // Tx
        cam_info_msg.P[5] = msg->cam_info[1];  // fy
        cam_info_msg.P[6] = msg->cam_info[3];  // cy
        cam_info_msg.P[7] = msg->cam_info[5];  // cy
        cam_info_msg.P[10] = 1.0;

        cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        cam_info_msg.width = static_cast<unsigned int>(image->rgb_image_.cols);
        cam_info_msg.height = static_cast<unsigned int>(image->rgb_image_.rows);
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info_msg);

        image->cam_model_ = cam_model;
        image->timestamp_ = msg->header.stamp.toSec();
        image->frame_id_ = msg->header.frame_id;
    }
    else if (msg->version == 2)
    {
        std::stringstream stream;
        tue::serialization::convert(msg->rgb, stream);
        tue::serialization::InputArchive a(stream);
        rgbd::deserialize(a, *image);
    }
    else if (msg->version == 3)
    {
        std::stringstream compressed, decompressed;
        tue::serialization::convert(msg->rgb, compressed);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(compressed);
        boost::iostreams::copy(in, decompressed);
        tue::serialization::InputArchive a(decompressed);
        rgbd::deserialize(a, *image);
    }
    else if (msg->version == 4)
    {
        std::stringstream compressed, decompressed;
        tue::serialization::convert(msg->rgb, compressed);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::zstd_decompressor());
        in.push(compressed);
        boost::iostreams::copy(in, decompressed);
        tue::serialization::InputArchive a(decompressed);
        rgbd::deserialize(a, *image);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("conversions", "convert: version '" << msg->version << "' not supported");
        return false;
    }

    return true;
}

}
