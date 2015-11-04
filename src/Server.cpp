#include "rgbd/Server.h"
#include "rgbd/RGBDMsg.h"
#include "rgbd/Image.h"
#include "rgbd/serialization.h"

#include <opencv2/highgui/highgui.hpp>

#include <tue/serialization/conversions.h>

#include <tue/profiling/timer.h>

#include <ros/node_handle.h>

namespace rgbd {

const int Server::SERIALIZATION_VERSION = 2;

// ----------------------------------------------------------------------------------------

Server::Server()
{
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
    send_thread_.join();
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type)
{
    ros::NodeHandle nh;
    pub_image_ = nh.advertise<rgbd::RGBDMsg>(name, 1);
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;

    nh.setCallbackQueue(&cb_queue_);
    service_server_ = nh.advertiseService(name, &Server::serviceServerCallback, this);

    // Initialize shared mem server
    shared_mem_server_.initialize(name);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool threaded)
{
    if (!threaded)
    {
        sendImpl(image);
        return;
    }

    send_thread_ = boost::thread(&Server::sendImpl, this, image);
}

// ----------------------------------------------------------------------------------------

void Server::sendImpl(const Image& image)
{    
    image_copy_ = image.clone();

    // Handle the service
    cb_queue_.callAvailable();

    // Send image using shared memory
    {
        boost::mutex::scoped_lock lock(send_mutex_shared_, boost::try_to_lock);
        if (!lock)
            return;

        shared_mem_server_.send(image_copy_);
    }

    if (pub_image_.getNumSubscribers() == 0)
        return;

    rgbd::RGBDMsg msg;
    msg.version = SERIALIZATION_VERSION;

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    serialize(image_copy_, a, rgb_type_, depth_type_);
    tue::serialization::convert(stream, msg.rgb);

    {
        boost::mutex::scoped_lock lock(send_mutex_topic_, boost::try_to_lock);
        if (!lock)
            return;

        pub_image_.publish(msg);
    }
}

bool Server::serviceServerCallback(GetRGBDRequest& req, GetRGBDResponse& resp)
{
    //! Check for valid input
    if (req.compression != GetRGBDRequest::JPEG && req.compression != GetRGBDRequest::PNG)
    {
        ROS_ERROR("Invalid compression, only JPEG and PNG are supported (see ENUM in srv definition)");
        return false;
    }

    //! Create resized images
    cv::Mat resized_rgb, resized_depth;

    double ratio_rgb = (double) req.width / image_copy_.getRGBImage().cols;
    double ratio_depth = (double) req.width / image_copy_.getDepthImage().cols;

    cv::resize(image_copy_.getRGBImage(), resized_rgb, cv::Size(req.width, image_copy_.getRGBImage().rows * ratio_rgb));
    cv::resize(image_copy_.getDepthImage(), resized_depth, cv::Size(req.width, image_copy_.getDepthImage().rows * ratio_depth));

    // Compress images
    std::string compression_str = req.compression == GetRGBDRequest::JPEG ? ".jpeg" : ".png";
    if (cv::imencode(compression_str, resized_rgb, resp.rgb_data) && cv::imencode(compression_str, resized_depth, resp.depth_data))
        return true;

    ROS_ERROR_STREAM("cv::imencode with compression_str " << compression_str << " failed!");

    return false;
}

}
