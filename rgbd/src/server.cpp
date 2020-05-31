#include "rgbd/server.h"
#include "rgbd/tools.h"

#include <ros/console.h>

#include <std_msgs/String.h>

#include <exception>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Server::Server()
{
    const std::string& hostname = get_hostname();
    if (hostname.empty())
    {
        ROS_FATAL("Can't determine hostname");
        throw std::runtime_error("Can't determine hostname");
    }
    hostname_ = hostname;
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
    nh_.shutdown();
    pub_hostname_thread_.join();
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type, const float service_freq)
{
    server_rgbd_.initialize(name, rgb_type, depth_type, service_freq);
    server_shm_.initialize(name);

    pub_shm_hostname_ = nh_.advertise<std_msgs::String>(name + "/hosts", 1);

    pub_hostname_thread_ = std::thread(&Server::pubHostnameThreadFunc, this, 10);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool)
{
    server_rgbd_.send(image);
    server_shm_.send(image);
}

// ----------------------------------------------------------------------------------------

void Server::pubHostnameThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    std_msgs::String msg;
    msg.data = hostname_;
    while(nh_.ok())
    {
        pub_shm_hostname_.publish(msg);
        r.sleep();
    }
}

}
