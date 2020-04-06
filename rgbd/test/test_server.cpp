#include "rgbd/Server.h"

#include <ros/init.h>
#include <ros/rate.h>

#include <image_geometry/pinhole_camera_model.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_server");

    rgbd::Server server;
    server.initialize("test", rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS);

    ros::Rate r(30);
    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
    cv::Mat depth_image(480, 640, CV_32FC1, 5.0);
    image_geometry::PinholeCameraModel cam_model;

    rgbd::Image image(rgb_image, depth_image, cam_model, "test_frame_id", ros::Time::now().toSec());

    int x = 0;
    while (ros::ok())
    {
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(0,0,255));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 5.0);
        x = (x + 10) % rgb_image.cols;
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(255, 0, 0));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 1.0);

        image.setRGBImage(rgb_image);
        image.setDepthImage(depth_image);
        image.setTimestamp(ros::Time::now().toSec());

        rgbd::Image image2(rgb_image, depth_image, cam_model, "test_frame_id", ros::Time::now().toSec());

        server.send(image, true);

        r.sleep();
    }

    return 0;
}
