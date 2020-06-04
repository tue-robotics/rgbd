#include "rgbd/server.h"
#include "rgbd/tools.h"

#include <std_msgs/String.h>

#include <functional>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Server::Server()
{
    const std::string& hostname = get_hostname();
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

    pub_hostname_thread_ = std::thread(rgbd::pubHostnameThreadFunc, std::ref(nh_), name, hostname_, 10);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool)
{
    server_rgbd_.send(image);
    server_shm_.send(image);
}

}
