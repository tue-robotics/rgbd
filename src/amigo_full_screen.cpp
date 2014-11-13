#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");

    rgbd::Client client;
//    client.intialize("rgbd");
    client.intialize("/amigo/top_kinect/rgbd");

    cv::namedWindow("AMIGO VIEW", CV_WINDOW_NORMAL);
    cv::setWindowProperty("AMIGO VIEW", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    ros::Rate r(30);
    while (ros::ok())
    {
        rgbd::Image image;
        if (client.nextImage(image))
        {
//            // Show depth image
//            if (image.getDepthImage().data)
//                cv::imshow("depth_original", image.getDepthImage() / 8);

            // Show rgb image
            if (image.getRGBImage().data)
                cv::imshow("AMIGO VIEW", image.getRGBImage());

            cv::waitKey(3);
        }

        r.sleep();
    }

    ros::spin();

    return 0;
}
