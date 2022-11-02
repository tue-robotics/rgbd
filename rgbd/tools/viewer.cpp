#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include "rgbd/client.h"
#include "rgbd/view.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    rgbd::Client client;
    client.initialize(ros::names::resolve("rgbd"));

    rgbd::Image image;

    ros::WallTime last_master_check = ros::WallTime::now();

    ros::Rate r(rate);
    while (ros::ok())
    {
        if (ros::WallTime::now() >= last_master_check + ros::WallDuration(1))
        {
            last_master_check = ros::WallTime::now();
            if (!ros::master::check())
            {
                ROS_FATAL("Lost connection to master");
                return 1;
            }
        }
        if (client.nextImage(image))
        {
            // Show depth image
            if (image.getDepthImage().data)
                cv::imshow("depth_original", image.getDepthImage() / 8);

            // Show rgb image
            if (image.getRGBImage().data)
                cv::imshow("rgb_original", image.getRGBImage());

            // Show combined image
            if (image.getDepthImage().data && image.getRGBImage().data)
            {
                cv::Mat image_hsv;
                cv::cvtColor(image.getRGBImage(), image_hsv, cv::COLOR_BGR2HSV);

                rgbd::View view(image, image_hsv.cols);

                cv::Mat canvas_hsv(view.getHeight(), view.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

                for(int y = 0; y < view.getHeight(); ++y)
                {
                    for(int x = 0; x < view.getWidth(); ++x)
                    {
                        float d = view.getDepth(x, y);
                        if (d == d)
                        {
                            cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(y, x);
                            hsv[2] = 255 - static_cast<unsigned char>(d / 8 * 255);
                            canvas_hsv.at<cv::Vec3b>(y, x) = hsv;
                        }
                    }
                }

                cv::Mat canvas_bgr;
                cv::cvtColor(canvas_hsv, canvas_bgr, cv::COLOR_HSV2BGR);
                cv::imshow("image + depth", canvas_bgr);
            }

            cv::waitKey(3);
        }

        r.sleep();
    }

    return 0;
}
