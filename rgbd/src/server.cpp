#include "rgbd/server.h"
#include "rgbd/tools.h"

#include <std_msgs/String.h>

#include <functional>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Server::Server() : pub_hostname_thread_ptr_(nullptr)
{
    const std::string& hostname = get_hostname();
    hostname_ = hostname;
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
    ROS_WARN("Server::~Server()");
    nh_.shutdown();
    if (pub_hostname_thread_ptr_)
        pub_hostname_thread_ptr_->join();
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type, const float service_freq)
{
    name_= name;
    server_rgbd_.initialize(name_, rgb_type, depth_type, service_freq);
    server_shm_.initialize(name_);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool)
{
    if (!pub_hostname_thread_ptr_)
        pub_hostname_thread_ptr_ = std::unique_ptr<std::thread>(new std::thread(rgbd::pubHostnameThreadFunc, std::ref(nh_), name_, hostname_, 20));
    server_rgbd_.send(image);
    server_shm_.send(image);
}

}
