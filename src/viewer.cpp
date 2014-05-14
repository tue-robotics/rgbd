#include "rgbd_transport/Client.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_viewer");

    rgbd::Client client;
    client.intialize("rgbd");

    ros::Rate r(30);
    while (ros::ok()) {
        rgbd::RGBDImage image;
        if (client.nextImage(image)) {
            cv::imshow("depth", image.getDepthImage() / 8);
            cv::imshow("rgb", image.getRGBImage());
            cv::waitKey(3);
        }
        r.sleep();
    }

    ros::spin();

    return 0;
}
