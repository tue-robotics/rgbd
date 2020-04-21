#include "rgbd/client_rgbd.h"

#include "rgbd/types.h"
#include "rgbd/ros/conversions.h"

namespace rgbd {

// ----------------------------------------------------------------------------------------

ClientRGBD::ClientRGBD()
{
}

// ----------------------------------------------------------------------------------------

ClientRGBD::~ClientRGBD()
{
}

// ----------------------------------------------------------------------------------------

void ClientRGBD::intialize(const std::string& server_name)
{
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd_msgs::RGBD>(
                server_name, 1, boost::bind(&ClientRGBD::rgbdImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh_.subscribe(sub_options);
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
