#ifndef TEST_SERVER_TEMPL_H_
#define TEST_SERVER_TEMPL_H_

#include <image_geometry/pinhole_camera_model.h>

#include "rgbd/image.h"

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

/**
 * Template function to test the communication provide by a server.
 * It generates artificial images and sends these by the provide server.
 * The server should have a
 * @code
 * bool initialize(std::string servername)
 * @endcode
 * and a
 * @code
 * void send(const rgbd::Image& image)
 * @endcode
 * function.
 */
template<class T>
int main_templ(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_server");
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    T server;
    server.initialize(ros::names::resolve("test"));

    ros::Rate r(rate);
    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
    cv::Mat depth_image(480, 640, CV_32FC1, 5.0);
    sensor_msgs::CameraInfo cam_info;
    cam_info.K = {554.2559327880068, 0.0, 320.5,
                  0.0, 554.2559327880068, 240.5,
                  0.0, 0.0, 1.0};
    cam_info.P = {554.2559327880068, 0.0, 320.5, 0.0,
                  0.0, 554.2559327880068, 240.5, 0.0,
                  0.0, 0.0, 1.0, 0.0};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = 640;
    cam_info.height = 480;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);

    rgbd::Image image(rgb_image, depth_image, cam_model, "test_frame_id", ros::Time::now().toSec());

    ros::WallTime last_master_check = ros::WallTime::now();

    int x = 0;
    while (ros::ok())
    {
        if (ros::WallTime::now() >= last_master_check + ros::WallDuration(1))
        {
            last_master_check = ros::WallTime::now();
            if (!ros::master::check())
            {
                ROS_ERROR("Lost connection to master");
                return 1;
            }
        }
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(0,0,255));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 5.0);
        x = (x + 10) % rgb_image.cols;
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(255, 0, 0));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 1.0);

        image.setRGBImage(rgb_image);
        image.setDepthImage(depth_image);
        image.setTimestamp(ros::Time::now().toSec());

        server.send(image);

        r.sleep();
    }

    return 0;
}


#endif // TEST_SERVER_TEMPL_H_
