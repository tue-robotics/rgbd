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
            cv::imshow("depth_original", image.getOriginalDepthImage() / 8);
            cv::imshow("rgb_original", image.getOriginalRGBImage());

            cv::Mat image_hsv;
            cv::cvtColor(image.getOriginalRGBImage(), image_hsv, CV_BGR2HSV);

            cv::Mat canvas_hsv(image.getHeight(), image.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));
\
            for(int y = 0; y < image.getHeight(); ++y)
            {
                for(int x = 0; x < image.getWidth(); ++x)
                {
                    float d = image.getDepth(x, y);
                    if (d == d)
                    {
                        cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(y, x);
                        hsv[2] = 255 - (d / 8 * 255);
                        canvas_hsv.at<cv::Vec3b>(y, x) = hsv;
                    }
                }
            }

            cv::Mat canvas_bgr;
            cv::cvtColor(canvas_hsv, canvas_bgr, CV_HSV2BGR);
            cv::imshow("image + depth", canvas_bgr);
            cv::waitKey(3);
        }

        r.sleep();
    }

    ros::spin();

    return 0;
}
