#include "rgbd/client.h"

#include <opencv2/core/mat.hpp>

#include "rgbd/image.h"
#include "rgbd/ros/conversions.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client()
{
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
}

// ----------------------------------------------------------------------------------------

bool Client::intialize(const std::string& server_name, float timeout)
{
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

}