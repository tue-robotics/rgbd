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

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type)
{
    server_rgbd_.initialize(name, rgb_type, depth_type);
    server_shm_.initialize(name);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool threaded)
{
    server_rgbd_.send(image, threaded);
    server_shm_.send(image);

}

}
