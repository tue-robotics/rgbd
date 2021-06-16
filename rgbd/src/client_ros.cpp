#include "rgbd/client_ros.h"
#include "rgbd/image.h"

// ROS message serialization
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace rgbd {

// ----------------------------------------------------------------------------------------

ClientROS::ClientROS() : ClientROSBase(ros::NodeHandle())
{
}

// ----------------------------------------------------------------------------------------

ClientROS::~ClientROS()
{
}

// ----------------------------------------------------------------------------------------

bool ClientROS::initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    nh_.setCallbackQueue(&cb_queue_);

    if (!ClientROSBase::initialize(rgb_image_topic, depth_image_topic, cam_info_topic))
        return false;

    sync_->registerCallback(boost::bind(&ClientROS::imageCallback, this, _1, _2));

    return true;
}

// ----------------------------------------------------------------------------------------

bool ClientROS::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return new_image_;
}

// ----------------------------------------------------------------------------------------

ImagePtr ClientROS::nextImage()
{
    image_ptr_ = nullptr;
    cb_queue_.callAvailable();
    return ImagePtr(image_ptr_);
}

}
