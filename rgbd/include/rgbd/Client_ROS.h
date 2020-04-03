/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_H_
#define RGBD_CLIENT_ROS_H_

#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "rgbd/types.h"


namespace rgbd {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDApproxPolicy;

class ClientROS {

public:

    ClientROS();

    virtual ~ClientROS();

    void intialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);

    bool initialized() { return (sync_); }

    bool nextImage(Image& image);

    ImagePtr nextImage();

protected:

    ros::NodeHandle nh_;
    ros::CallbackQueue cb_queue_;

    message_filters::Synchronizer<RGBDApproxPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_rgb_sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_depth_sync_;
    ros::Subscriber sub_cam_info_;
    image_geometry::PinholeCameraModel cam_model_;

    bool new_image_;
    Image* image_ptr_;

    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg);

};

}

#endif // RGBD_CLIENT_ROS_H_
