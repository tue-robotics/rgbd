#include <opencv2/highgui/highgui.hpp>

#include <ros/init.h>
#include <ros/names.h>
#include <ros/rate.h>

#include "rgbd/client.h"
#include "rgbd/image.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_test_client");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize(ros::names::resolve("test"));

    rgbd::Image image;

    ros::Rate r(30);
    while (ros::ok())
    {
        if (client.nextImage(image))
        {
            std::cout << "Image: t = " << std::fixed << image.getTimestamp() << ", frame = " << image.getFrameId() << std::endl;

            cv::imshow("rgb", image.getRGBImage());
            cv::imshow("depth", image.getDepthImage() / 8);
            cv::waitKey(3);
        }
        r.sleep();
    }

    return 0;
}
