/**
 * This client can either be used to listen to RGBD messages on a single topic
 * (use initialize(_)) or to seperate RGB / Depth / camera info topics
 * (use initialize(_, _, _)).
 *
 * The client will always convert the incoming data to rgbd::Image data structures.
 */

#ifndef RGBD_TRANSPORT_CLIENT_H_
#define RGBD_TRANSPORT_CLIENT_H_

#include <ros/ros.h>

#include "rgbd/RGBDMsg.h"
#include <ros/callback_queue.h>

#include "rgbd/types.h"
#include "rgbd/shared_mem_client.h"


// ROS message serialization
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace rgbd {

// Forward declare struct that holds all approximate time sync stuff
struct ROSImageSyncData;

class Image;

class Client {

public:

    Client();

    virtual ~Client();

    void intialize(const std::string& server_name);

    void intialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);

    bool initialized() { return !sub_image_.getTopic().empty() || ros_image_sync_data_; }

    bool nextImage(Image& image);

    ImagePtr nextImage();

protected:

    SharedMemClient shared_mem_client_;

    ros::NodeHandle* nh_;

    ros::Subscriber sub_image_;
    ros::CallbackQueue cb_queue_;

    bool received_image_;
    Image* image_ptr_;

    ROSImageSyncData* ros_image_sync_data_;

    void rgbdImageCallback(const rgbd::RGBDMsg::ConstPtr& msg);

    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg);

};

}

#endif
