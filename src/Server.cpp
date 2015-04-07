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
    rgbd::Image image_copy = image.clone();

    // Send image using shared memory
    {
        boost::mutex::scoped_lock lock(send_mutex_shared_, boost::try_to_lock);
        if (!lock)
            return;

        shared_mem_server_.send(image_copy);
    }

    if (pub_image_.getNumSubscribers() == 0)
        return;

    rgbd::RGBDMsg msg;
    msg.version = SERIALIZATION_VERSION;

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    serialize(image_copy, a, rgb_type_, depth_type_);
    tue::serialization::convert(stream, msg.rgb);

    {
        boost::mutex::scoped_lock lock(send_mutex_topic_, boost::try_to_lock);
        if (!lock)
            return;

        pub_image_.publish(msg);
    }
}

}
