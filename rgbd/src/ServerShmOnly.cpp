#include "rgbd/ServerShmOnly.h"
#include "rgbd_msgs/RGBD.h"
#include "rgbd/Image.h"
#include "rgbd/serialization.h"
#include "rgbd/ros/conversions.h"

#include <opencv2/highgui/highgui.hpp>

#include <tue/serialization/conversions.h>

#include <ros/node_handle.h>

namespace rgbd {

// ----------------------------------------------------------------------------------------

ServerShmOnly::ServerShmOnly()
{
}

// ----------------------------------------------------------------------------------------

ServerShmOnly::~ServerShmOnly()
{
    send_thread_.join();
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type)
{
    ros::NodeHandle nh;
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;

    // Initialize shared mem server
    shared_mem_server_.initialize(name);
}

void ServerShmOnly::rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg)
{
    if (convert(msg, image_ptr_))
        send(*image_ptr_);
    else
        std::cout << "Could not convert message to Image" << std::endl;
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::send(const Image& image, bool threaded)
{
    if (!threaded)
    {
        sendImpl(image);
        return;
    }

    send_thread_ = boost::thread(&ServerShmOnly::sendImpl, this, image);
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::sendImpl(const Image& image)
{    
    // Send image using shared memory
    boost::mutex::scoped_lock lock(send_mutex_shared_, boost::try_to_lock);
    if (!lock)
        return;

    shared_mem_server_.send(image);
}

}
