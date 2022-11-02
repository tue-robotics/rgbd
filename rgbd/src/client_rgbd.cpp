#include "rgbd/client_rgbd.h"

#include "rgbd/types.h"
#include "rgbd/ros/conversions.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

ClientRGBD::ClientRGBD() : image_ptr_(nullptr)
{
}

// ----------------------------------------------------------------------------------------

ClientRGBD::~ClientRGBD()
{
}

// ----------------------------------------------------------------------------------------

bool ClientRGBD::initialize(const std::string& server_name)
{
    ros::NodeHandle nh;
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd_msgs::RGBD>(
                server_name, 1, boost::bind(&ClientRGBD::rgbdImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh.subscribe(sub_options);

    return true;
}

// ----------------------------------------------------------------------------------------

bool ClientRGBD::deinitialize()
{
    sub_image_ = ros::Subscriber(); // Old subscriber is deleted, so it unsubscribes. New subsriber is not subscribed to anything.
    return true;
}

// ----------------------------------------------------------------------------------------

bool ClientRGBD::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return new_image_;
}

// ----------------------------------------------------------------------------------------

ImagePtr ClientRGBD::nextImage()
{
    image_ptr_ = nullptr;
    cb_queue_.callAvailable();
    return ImagePtr(image_ptr_);
}

// ----------------------------------------------------------------------------------------

void ClientRGBD::rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg)
{
    new_image_ = convert(msg, image_ptr_);
}

}
