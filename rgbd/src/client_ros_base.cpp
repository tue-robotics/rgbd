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

ClientROSBase::ClientROSBase(ros::NodeHandle nh) : nh_(nh), sync_(nullptr), sub_rgb_sync_(nullptr), sub_depth_sync_(nullptr), image_ptr_(nullptr)
{
}

// ----------------------------------------------------------------------------------------

ClientROSBase::~ClientROSBase()
{
    deinitialize();
}

// ----------------------------------------------------------------------------------------

bool ClientROSBase::initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    sub_cam_info_ = nh_.subscribe(cam_info_topic, 1, &ClientROSBase::camInfoCallback, this);

    sub_rgb_sync_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> >(new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgb_image_topic, 1));
    sub_depth_sync_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> >(new message_filters::Subscriber<sensor_msgs::Image>(nh_, depth_image_topic, 1));

    sync_ = std::unique_ptr<message_filters::Synchronizer<RGBDApproxPolicy> >(new message_filters::Synchronizer<RGBDApproxPolicy>(RGBDApproxPolicy(10), *sub_rgb_sync_, *sub_depth_sync_));

    return true;
}

// ----------------------------------------------------------------------------------------

bool ClientROSBase::deinitialize()
{
    nh_.shutdown();
    sync_.reset();
    sub_rgb_sync_.reset();
    sub_depth_sync_.reset();
    return true;
}

// ----------------------------------------------------------------------------------------

void ClientROSBase::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    if (!cam_model_.initialized())
    {
        cam_model_.fromCameraInfo(cam_info_msg);
        sub_cam_info_.shutdown();
    }
    else
    {
        ROS_ERROR_NAMED("ClientROS", "CameraInfo should unsubsribe after inititializing the camera model");
    }
}

// ----------------------------------------------------------------------------------------

bool ClientROSBase::imageCallback(const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::ImageConstPtr& depth_image_msg)
{
    if (!cam_model_.initialized())
    {
        ROS_ERROR_DELAYED_THROTTLE_NAMED(1, "ClientROS", "ClientROSBase: cam_model not yet initialized");
        return false;
    }
    cv_bridge::CvImagePtr rgb_img_ptr, depth_img_ptr;

    // Convert RGB image
    try
    {
        rgb_img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_NAMED("ClientROS", "ClientROSBase: Could not deserialize rgb image: %s", e.what());
        return false;
    }

    // Convert depth image
    try
    {
        // cv_bridge doesn't support changing the encoding of depth images, so just creating ImagePtr
        depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, depth_image_msg->encoding);

        if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
            cv::Mat depth_image(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1);
            for(int x = 0; x < depth_image.cols; ++x)
            {
                for(int y = 0; y < depth_image.rows; ++y)
                {
                    depth_image.at<float>(y, x) = static_cast<float>(depth_img_ptr->image.at<unsigned short>(y, x)) / 1000; // (mm to m)
                }
            }

            depth_img_ptr->image = depth_image;
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_NAMED("ClientROS", "ClientROSBase: Could not deserialize depth image: %s", e.what());
        return false;
    }

    if (!image_ptr_)
        // in this case, the pointer will always be wrapped in a shared ptr, so no mem leaks (see nextImage() )
        image_ptr_ = new Image();

    image_ptr_->setRGBImage(rgb_img_ptr->image);
    image_ptr_->setDepthImage(depth_img_ptr->image);
    image_ptr_->setCameraModel(cam_model_);
    image_ptr_->setFrameId(rgb_image_msg->header.frame_id);
    image_ptr_->setTimestamp(rgb_image_msg->header.stamp.toSec());
    new_image_ = true;

    return true;
}

}
