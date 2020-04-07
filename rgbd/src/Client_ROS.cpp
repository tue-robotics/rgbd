#include "rgbd/Client_ROS.h"
#include "rgbd/Image.h"

// ROS message serialization
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace rgbd {

// ----------------------------------------------------------------------------------------

ClientROS::ClientROS() : sync_(nullptr), sub_rgb_sync_(nullptr), sub_depth_sync_(nullptr), image_ptr_(nullptr)
{
}

// ----------------------------------------------------------------------------------------

ClientROS::~ClientROS()
{
    nh_.shutdown();
    delete sub_rgb_sync_;
    delete sub_depth_sync_;
    delete sync_;
}

// ----------------------------------------------------------------------------------------

void ClientROS::intialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    nh_.setCallbackQueue(&cb_queue_);

    sub_cam_info_ = nh_.subscribe(cam_info_topic, 1, &ClientROS::camInfoCallback, this);

    sub_rgb_sync_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgb_image_topic, 1);
    sub_depth_sync_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, depth_image_topic, 1);

    sync_ = new message_filters::Synchronizer<RGBDApproxPolicy>(RGBDApproxPolicy(10), *sub_rgb_sync_, *sub_depth_sync_);
    sync_->registerCallback(boost::bind(&ClientROS::imageCallback, this, _1, _2));
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

// ----------------------------------------------------------------------------------------

void ClientROS::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    if (!cam_model_.initialized())
        cam_model_.fromCameraInfo(cam_info_msg);
}

// ----------------------------------------------------------------------------------------

void ClientROS::imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg)
{
    cv_bridge::CvImagePtr img_ptr, depth_img_ptr;

    // Convert RGB image
    try
    {
        img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not deserialize rgb image: %s", e.what());
        return;
    }

    // Convert depth image
    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, depth_image_msg->encoding);

        if (depth_image_msg->encoding == "16UC1")
        {
            // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
            cv::Mat depth_image(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1);
            for(int x = 0; x < depth_image.cols; ++x)
            {
                for(int y = 0; y < depth_image.rows; ++y)
                {
                    depth_image.at<float>(y, x) = ((float)depth_img_ptr->image.at<unsigned short>(y, x)) / 1000; // (mm to m)
                }
            }

            depth_img_ptr->image = depth_image;
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not deserialize depth image: %s", e.what());
        return;
    }

    if (!image_ptr_)
        // in this case, the pointer will always be wrapped in a shared ptr, so no mem leaks (see nextImage() )
        image_ptr_ = new Image();

    image_ptr_->rgb_image_ = img_ptr->image;
    image_ptr_->depth_image_ = depth_img_ptr->image;
    image_ptr_->cam_model_ = cam_model_;
    image_ptr_->frame_id_ = rgb_image_msg->header.frame_id;
    image_ptr_->timestamp_ = rgb_image_msg->header.stamp.toSec();
    new_image_ = true;
}

}
