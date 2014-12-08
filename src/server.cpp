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

#include "rgbd/Image.h"
#include "rgbd/Server.h"

#include <image_geometry/pinhole_camera_model.h>

image_geometry::PinholeCameraModel cam_model_;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectApproxPolicy;

rgbd::Server rgbd_server;

// ----------------------------------------------------------------------------------------

void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg) {
    if (!cam_model_.initialized()) {
        return;
    }

    cv_bridge::CvImagePtr img_ptr, depth_img_ptr;

    // Convert RGB image
    try {
        img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not deserialize rgb image: %s", e.what());
        return;
    }

    // Convert depth image
    try {
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

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not deserialize depth image: %s", e.what());
        return;
    }

    rgbd::Image image(img_ptr->image,depth_img_ptr->image,cam_model_,rgb_image_msg->header.frame_id,rgb_image_msg->header.stamp.toSec());

    rgbd_server.send(image);
}

// ----------------------------------------------------------------------------------------

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
    if (!cam_model_.initialized()) {
        cam_model_.fromCameraInfo(cam_info_msg);
    }
}

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_server");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double max_fps = 30;
    nh_private.getParam("max_fps", max_fps);

    // ----- READ RGB STORAGE TYPE

    rgbd::RGBStorageType rgb_type;

    std::string rgb_type_str = "lossless";
    nh_private.getParam("rgb_storage", rgb_type_str);
    if (rgb_type_str == "none")
        rgb_type = rgbd::RGB_STORAGE_NONE;
    else if (rgb_type_str == "lossless")
        rgb_type = rgbd::RGB_STORAGE_LOSSLESS;
    else if (rgb_type_str == "jpg")
        rgb_type = rgbd::RGB_STORAGE_JPG;
    else
    {
        ROS_ERROR("Unknown 'rgb_storage' type: should be 'none', 'lossless', or 'jpg'.");
        return 1;
    }

    // ----- READ DEPTH STORAGE TYPE

    rgbd::DepthStorageType depth_type;

    std::string depth_type_str = "lossless";
    nh_private.getParam("depth_storage", depth_type_str);
    if (depth_type_str == "none")
        depth_type = rgbd::DEPTH_STORAGE_NONE;
    else if (depth_type_str == "lossless")
        depth_type = rgbd::DEPTH_STORAGE_LOSSLESS;
    else if (depth_type_str == "png")
        depth_type = rgbd::DEPTH_STORAGE_PNG;
    else
    {
        ROS_ERROR("Unknown 'depth_storage' type: should be 'none', 'lossless', or 'png'.");
        return 1;
    }

    // ------------------------------

    rgbd_server.initialize("output", rgb_type, depth_type);

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
