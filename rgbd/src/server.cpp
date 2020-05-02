#include "rgbd/server.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

Server::Server()
{
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type, const float service_freq)
{
    server_rgbd_.initialize(name, rgb_type, depth_type, service_freq);
    server_shm_.initialize(name);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool)
{
    server_rgbd_.send(image);
    server_shm_.send(image);

}

}
