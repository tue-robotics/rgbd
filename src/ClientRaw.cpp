#include "rgbd/ClientRaw.h"
#include <cv_bridge/cv_bridge.h>

namespace rgbd {

ClientRaw::ClientRaw() {
}

ClientRaw::~ClientRaw() {
}

// ---------------------------------------------------------------------------------------------

void ClientRaw::setDepthTopic(const std::string& topic) {
    ros::NodeHandle nh;

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::Image>(
                topic, 1, boost::bind(&ClientRaw::depthImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_cam_info_ = nh.subscribe(sub_options);
}

// ---------------------------------------------------------------------------------------------

void ClientRaw::setCameraInfoTopic(const std::string& topic) {
    ros::NodeHandle nh;

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(
                topic, 1, boost::bind(&ClientRaw::cameraInfoCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh.subscribe(sub_options);
}

// ---------------------------------------------------------------------------------------------

bool ClientRaw::nextImage(Image& image) {
    received_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return received_image_;
}

// ---------------------------------------------------------------------------------------------

void ClientRaw::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
    if (!cam_model_.initialized()) {
        cam_model_.fromCameraInfo(cam_info_msg);
    }
}

// ---------------------------------------------------------------------------------------------

void ClientRaw::depthImageCallback(sensor_msgs::ImageConstPtr depth_img_msg) {
//    if (!cam_model_.initialized()) {
//        return;
//    }

//    // Convert depth image
//    try {
//        cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(depth_img_msg, "32FC1");
//        image_ptr_->depth_image_ = depth_img_ptr->image;
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("Could not deserialize depth image: %s", e.what());
//    }

//    image_ptr_->setFrameID(depth_img_msg->header.frame_id);
//    image_ptr_->setTimestamp(depth_img_msg->header.stamp.toSec());
//    image_ptr_->setCameraModel(cam_model_);
//    received_image_ = true;
}

}
