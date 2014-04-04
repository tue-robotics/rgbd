#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Message filters
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>

// CV bridge
#include <cv_bridge/cv_bridge.h>

#include "rgbd_transport/RGBDImage.h"
#include "rgbd_transport/Server.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectApproxPolicy;

Server rgbd_server;

void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg) {
    RGBDImage image;
    image.setTimestamp(rgb_image_msg->header.stamp.toSec());
    image.setFrameID(rgb_image_msg->header.frame_id);

    // Convert RGB image
    try {
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
        image.setRGBImage(img_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not deserialize rgb image: %s", e.what());
        return;
    }

    // Convert depth image
    try {
        cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, "32FC1");
        image.setDepthImage(depth_img_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not deserialize depth image: %s", e.what());
        return;
    }

    rgbd_server.send(image);
}

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_server");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double max_fps = 30;
    nh_private.getParam("max_fps", max_fps);

    rgbd_server.initialize("/test");

    ros::Subscriber sub_cam_info = nh.subscribe("cam_info", 1, &camInfoCallback);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_sync_(nh, "rgb_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_sync_(nh, "depth_image", 1);

    message_filters::Synchronizer<KinectApproxPolicy> sync_(KinectApproxPolicy(10), sub_rgb_sync_, sub_depth_sync_);
    sync_.registerCallback(boost::bind(&imageCallback, _1, _2));

    ros::Rate r(max_fps);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
