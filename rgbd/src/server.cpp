#include "rgbd/server.h"
#include "rgbd/utility.h"

#include <std_msgs/String.h>

#include <functional>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Server::Server(ros::NodeHandlePtr nh) : nh_(nullptr), pub_hostname_thread_ptr_(nullptr)
{
    if (nh)
        nh_ = nh;
    else
        nh_ = boost::make_shared<ros::NodeHandle>();
    const std::string& hostname = get_hostname();
    hostname_ = hostname;
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
    nh_->shutdown();
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
    // Publisher is created here, because shared memory is opened as the first image is send. Because the image size is unknown before.
    // Creating the publisher earlier will cause the client to initialize the client_shm, while the shared memory doesn't exist yet. 
    if (!pub_hostname_thread_ptr_)
        pub_hostname_thread_ptr_ = std::make_unique<std::thread>(rgbd::pubHostnameThreadFunc, std::ref(nh_), name_, hostname_, 20);
    server_rgbd_.send(image);
    server_shm_.send(image);
}

}
