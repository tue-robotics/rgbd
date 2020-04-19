#include "rgbd/Client.h"

#include <opencv2/core/mat.hpp>

#include "rgbd/Image.h"
#include "rgbd/ros/conversions.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client() : nh_(nullptr)
{
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
    sub_image_.shutdown();
    delete nh_;
}

// ----------------------------------------------------------------------------------------

void Client::intialize(const std::string& server_name, float timeout)
{
    if (shared_mem_client_.intialize(server_name, timeout))
        return;

    // If the shared memory client could not be created, use ROS topics instead

    delete nh_;
    nh_ = new ros::NodeHandle();

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd_msgs::RGBD>(
                server_name, 1, boost::bind(&Client::rgbdImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh_->subscribe(sub_options);
}

// ----------------------------------------------------------------------------------------

bool Client::nextImage(Image& image)
{
    if (shared_mem_client_.initialized())
        return shared_mem_client_.nextImage(image);

    new_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return new_image_;
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage()
{
    if (shared_mem_client_.initialized())
    {
        ImagePtr img(new Image);
        if (shared_mem_client_.nextImage(*img))
            return img;
        else
            return ImagePtr();
    }

    image_ptr_ = nullptr;
    cb_queue_.callAvailable();
    return ImagePtr(image_ptr_);
}

// ----------------------------------------------------------------------------------------

void Client::rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg)
{
    new_image_ = convert(msg, image_ptr_);
}

}
