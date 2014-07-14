#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_test_client");

    rgbd::Client client;
    client.intialize("test");

    ros::Rate r(30);
    while (ros::ok()) {
        rgbd::Image image;
        if (client.nextImage(image)) {
            rgbd::View view(image,640);
            cv::imshow("depth", image.getDepthImage() / 8);
            cv::waitKey(3);
        }
        r.sleep();
    }

    ros::spin();

    return 0;
}
