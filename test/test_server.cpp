#include "rgbd/Server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_server");

    rgbd::Server server;
    server.initialize("test", rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS);

    ros::Rate r(30);
    while (ros::ok())
    {
        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
        cv::Mat depth_image(480, 640, CV_32FC1, 0.0);

        geo::DepthCamera cam_model;
        rgbd::Image image(rgb_image, depth_image, cam_model, "", 0);

        server.send(image);

        r.sleep();
    }

    ros::spin();

    return 0;
}
