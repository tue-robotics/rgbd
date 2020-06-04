#include "rgbd/client.h"

#include <opencv2/core/mat.hpp>

#include "rgbd/image.h"
#include "rgbd/ros/conversions.h"
#include "rgbd/tools.h"

#include <ros/duration.h>
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

    server_name_ = server_name;
    return true;
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
    if (msg->data != hostname_)
        return;

    last_time_shm_server_online_ = ros::WallTime::now();
    ROS_DEBUG_STREAM_THROTTLE(5, "SHM server online on: " << hostname_);
}

// ----------------------------------------------------------------------------------------

void Client::subHostsThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    ros::WallDuration d(r);
    while(nh_.ok())
    {
        cb_queue_.callAvailable();
        if (ros::WallTime::now() - d > last_time_shm_server_online_)
        {
            // No message received for more than one cycle time, so RGBD topic should be used
            ROS_ERROR_STREAM("Disable ClientSHM");
            if(!client_rgbd_.initialized())
            {
                ROS_DEBUG("Switching from ClientSHM to ClientRGBD");
                client_shm_.deintialize();
                client_rgbd_.intialize(server_name_);
            }
        }
        else
        {
            // Message received less than one cycle time ago, so shared memory should be used
            ROS_ERROR_STREAM("Enable ClientSHM");
            if (!client_shm_.initialized())
            {
                ROS_DEBUG("Switching from ClientRGBD to ClientSHM");
                client_rgbd_.deintialize();
                client_shm_.intialize(server_name_, 0);
            }
        }
        r.sleep();
    }
}

}
