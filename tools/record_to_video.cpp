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

#include <opencv2/highgui/highgui.hpp>

#include "rgbd/Image.h"
#include "rgbd/Server.h"

#include <image_geometry/pinhole_camera_model.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectApproxPolicy;

cv::Mat rgb_image;

// ----------------------------------------------------------------------------------------

void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg) {

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

    rgb_image = img_ptr->image;
}

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_server");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double video_frame_rate = 30;
    nh_private.getParam("fps", video_frame_rate);

    const cv::Size2i video_size(640, 480);
    cv::VideoWriter video_writer("/tmp/video.avi", CV_FOURCC('M','J','P','G'), video_frame_rate, video_size);

    if (!video_writer.isOpened())
    {
        std::cout << "Unable to create video writer" << std::endl;
        return 1;
    }

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_sync_(nh, "rgb_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_sync_(nh, "depth_image", 1);

    message_filters::Synchronizer<KinectApproxPolicy> sync_(KinectApproxPolicy(10), sub_rgb_sync_, sub_depth_sync_);
    sync_.registerCallback(boost::bind(&imageCallback, _1, _2));

    ros::Rate r(video_frame_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        if (rgb_image.data)
            video_writer.write(rgb_image);

        r.sleep();
    }

    video_writer.release();

    return 0;
}
