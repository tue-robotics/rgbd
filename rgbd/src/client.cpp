#include "rgbd/client.h"

#include <opencv2/core/mat.hpp>

#include "rgbd/image.h"
#include "rgbd/ros/conversions.h"
#include "rgbd/tools.h"

#include <ros/rate.h>

#include <std_msgs/String.h>


namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client()
{
    const std::string& hostname = get_hostname();
    hostname_ = hostname;
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
}

// ----------------------------------------------------------------------------------------

bool Client::intialize(const std::string& server_name, float timeout)
{
    nh_.setCallbackQueue(&cb_queue_);
    sub_shm_hosts_ = nh_.subscribe<std_msgs::String>(server_name + "/hosts", 1, &Client::hostsCallback, this);

    sub_hosts_thread_ = std::thread(&Client::subHostsThreadFunc, this, 10);

    if (client_shm_.intialize(server_name, timeout))
        return true;

    return client_rgbd_.intialize(server_name);
}

// ----------------------------------------------------------------------------------------

bool Client::nextImage(Image& image)
{
    if (client_shm_.initialized())
        return client_shm_.nextImage(image);

    return client_rgbd_.nextImage(image);
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage()
{
    if (client_shm_.initialized())
    {
        ImagePtr img(new Image);
        if (client_shm_.nextImage(*img))
            return img;
        else
            return ImagePtr();
    }

    return client_rgbd_.nextImage();
}

// ----------------------------------------------------------------------------------------

void Client::hostsCallback(const std_msgs::StringConstPtr& msg)
{
    if (msg->data == hostname_)
        ROS_ERROR_STREAM("SHM server online on: " << hostname_);
}

// ----------------------------------------------------------------------------------------

void Client::subHostsThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    while(nh_.ok())
    {
        cb_queue_.callAvailable();
        r.sleep();
    }
}

}
