#include "rgbd/client.h"

#include <opencv2/core/mat.hpp>

#include "rgbd/image.h"
#include "rgbd/ros/conversions.h"
#include "rgbd/tools.h"

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/rate.h>

#include <std_msgs/String.h>


namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client() : client_impl_mode_(ClientImplMode::rgbd)
{
    const std::string& hostname = get_hostname();
    hostname_ = hostname;
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
    nh_.shutdown();
    sub_hosts_thread_.join();
}

// ----------------------------------------------------------------------------------------

bool Client::initialize(const std::string& server_name, float /*timeout*/)
{
    nh_.setCallbackQueue(&cb_queue_);
    sub_shm_hosts_ = nh_.subscribe<std_msgs::String>(server_name + "/hosts", 1, &Client::hostsCallback, this);

    sub_hosts_thread_ = std::thread(&Client::subHostsThreadFunc, this, 20);

    server_name_ = server_name;
    return true;
}

// ----------------------------------------------------------------------------------------

bool Client::deinitialize()
{
    server_name_ = "";
    nh_.shutdown();
    if (client_shm_.initialized())
        client_shm_.deinitialize();
    if (client_rgbd_.initialized())
        client_rgbd_.deinitialize();
    last_time_shm_server_online_ = ros::WallTime();
    sub_hosts_thread_.join();
    return true;
}

// ----------------------------------------------------------------------------------------

bool Client::nextImage(Image& image)
{
    if (client_impl_mode_ == ClientImplMode::shm)
    {
        std::lock_guard<std::mutex> lg(switch_impl_mutex_);
        return client_shm_.nextImage(image);
    }
    else // client_impl_mode == ClientImplMode::rgbd
    {
        std::lock_guard<std::mutex> lg(switch_impl_mutex_);
        return client_rgbd_.nextImage(image);
    }
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage()
{
    if (client_impl_mode_ == ClientImplMode::shm)
    {
        std::lock_guard<std::mutex> lg(switch_impl_mutex_);
        return client_shm_.nextImage();
    }
    else // client_impl_mode == ClientImplMode::rgbd
    {
        std::lock_guard<std::mutex> lg(switch_impl_mutex_);
        return client_rgbd_.nextImage();
    }
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
    ros::WallRate r(frequency);
    ros::WallDuration d(5*r.expectedCycleTime().toSec()); // To prevent any issues by a very small delay
    while(nh_.ok())
    {
        cb_queue_.callAvailable();
        // Decide on implementation mode
        if (ros::WallTime::now() - d > last_time_shm_server_online_)
        {
            // Too much time has elapsed since last message, use rgbd
            // Lock the entire switching procedure, but not the decision;
            std::lock_guard<std::mutex> lg(switch_impl_mutex_);
            if(client_shm_.initialized())
                client_shm_.deinitialize();
            if(!client_rgbd_.initialized())
            {
                ROS_DEBUG("Switching to ClientRGBD");
                client_rgbd_.initialize(server_name_);
            }
            client_impl_mode_ = ClientImplMode::rgbd;
        }
        else
        {
            // Last message is recent enough to use shm
            std::lock_guard<std::mutex> lg(switch_impl_mutex_);
            if (client_rgbd_.initialized())
                client_rgbd_.deinitialize();
            if (!client_shm_.initialized())
            {
                ROS_DEBUG("Switching to ClientSHM");
                // It might take multiple iterations before ClientSHM is initialized.
                client_shm_.initialize(server_name_, 0);
            }
            client_impl_mode_ = ClientImplMode::shm;
        }
        r.sleep();
    }
}

}
