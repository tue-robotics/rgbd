#include "rgbd/server.h"

#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/utility.h"
#include <memory>
#include <rclcpp/node.hpp>
#include <string>

namespace rgbd
{

Server::Server(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_server")), server_rgbd_(node_), server_shm_(node_),
    hostname_(getHostname())
{
}

Server::~Server()
{
    if (pub_hostname_thread_ptr_ && pub_hostname_thread_ptr_->joinable())
    {
        pub_hostname_thread_ptr_->join();
    }
}

void Server::initialize(const std::string& name,
                        RGBStorageType rgb_type,
                        DepthStorageType depth_type,
                        float service_freq)
{
    name_ = name;
    server_rgbd_.initialize(name_, rgb_type, depth_type, service_freq);
    server_shm_.initialize(name_);
}

void Server::send(const Image& image, bool)
{
    if (!pub_hostname_thread_ptr_)
    {
        pub_hostname_thread_ptr_ =
            std::make_unique<std::thread>(rgbd::pubHostnameThreadFunc, node_, name_, hostname_, 20.0f);
    }
    server_rgbd_.send(image);
    server_shm_.send(image);
}

} // namespace rgbd
