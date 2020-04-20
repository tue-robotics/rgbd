#include "rgbd/Server_RGBD.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rgbd_msgs/RGBD.h>

#include <ros/node_handle.h>

#include <tue/serialization/conversions.h>

#include "rgbd/Image.h"
#include "rgbd/serialization.h"

namespace rgbd {

const int ServerRGBD::MESSAGE_VERSION = 2;

// ----------------------------------------------------------------------------------------

ServerRGBD::ServerRGBD()
{
}

// ----------------------------------------------------------------------------------------

ServerRGBD::~ServerRGBD()
{
    nh_.shutdown();
    send_thread_.join();
    service_thread_.join();
}

// ----------------------------------------------------------------------------------------

void ServerRGBD::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type, const float service_freq)
{
    pub_image_ = nh_.advertise<rgbd_msgs::RGBD>(name, 1);
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;

    nh_.setCallbackQueue(&cb_queue_);
    service_server_ = nh_.advertiseService(name, &ServerRGBD::serviceCallback, this);
    service_thread_ = boost::thread(&ServerRGBD::serviceThreadFunc, this, service_freq);
}

// ----------------------------------------------------------------------------------------

void ServerRGBD::send(const Image& image, bool threaded)
{
    threaded = false; // Running threaded causes a memory leak.
    if (!threaded)
    {
        sendImpl(image);
        return;
    }

    send_thread_ = boost::thread(&ServerRGBD::sendImpl, this, image);
}

// ----------------------------------------------------------------------------------------

void ServerRGBD::sendImpl(const Image& image)
{
    {
        boost::unique_lock<boost::mutex> ul(image_mutex_);
        image_ = image.clone();
    }

    if (pub_image_.getNumSubscribers() == 0)
        return;

    rgbd_msgs::RGBD msg;
    msg.version = MESSAGE_VERSION;

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    serialize(image, a, rgb_type_, depth_type_);
    tue::serialization::convert(stream, msg.rgb);

    pub_image_.publish(msg);
}

bool ServerRGBD::serviceCallback(rgbd_msgs::GetRGBDRequest& req, rgbd_msgs::GetRGBDResponse& resp)
{
    rgbd::Image image;
    {
        boost::unique_lock<boost::mutex> ul(image_mutex_);
        image = image_.clone();
    }
    // Check for valid input
    if (req.compression != rgbd_msgs::GetRGBDRequest::JPEG && req.compression != rgbd_msgs::GetRGBDRequest::PNG)
    {
        ROS_ERROR("Invalid compression, only JPEG and PNG are supported (see ENUM in srv definition)");
        return false;
    }

    // Create resized images
    cv::Mat resized_rgb, resized_depth;

    double ratio_rgb = static_cast<double>(req.width / image.getRGBImage().cols);
    double ratio_depth = static_cast<double>( req.width / image.getDepthImage().cols);

    cv::resize(image.getRGBImage(), resized_rgb, cv::Size(req.width, static_cast<int>(image.getRGBImage().rows * ratio_rgb)));
    cv::resize(image.getDepthImage(), resized_depth, cv::Size(req.width, static_cast<int>(image.getDepthImage().rows * ratio_depth)));

    // Compress images
    std::string compression_str = req.compression == rgbd_msgs::GetRGBDRequest::JPEG ? ".jpeg" : ".png";
    if (cv::imencode(compression_str, resized_rgb, resp.rgb_data) && cv::imencode(compression_str, resized_depth, resp.depth_data))
        return true;

    ROS_ERROR_STREAM("cv::imencode with compression_str " << compression_str << " failed!");

    return false;
}

void ServerRGBD::serviceThreadFunc(const float freq)
{
    ros::Rate r(static_cast<double>(freq));
    while(nh_.ok())
    {
        cb_queue_.callAvailable();
        r.sleep();
    }
}

}
