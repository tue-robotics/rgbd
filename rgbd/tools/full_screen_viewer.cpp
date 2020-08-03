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
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    rgbd::Client client;
    client.initialize(ros::names::resolve("rgbd"));

    const std::string  window_name = "RGBD_VIEW";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    bool PAUSE = false;

    cv::Mat canvas;

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
                ROS_ERROR("Lost connection to master");
                return 1;
            }
        }
        if (!PAUSE && client.nextImage(image))
        {
            // Show rgb image
            if (image.getRGBImage().data)
                canvas = image.getRGBImage();
        }

        if (PAUSE)
            cv::putText(canvas, "PAUSED", cv::Point(10, canvas.rows - 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        cv::imshow(window_name, canvas);

        int i_key = cv::waitKey(3);
        if (i_key >= 0)
        {
            char key = static_cast<char>(i_key);

            if (key == ' ')
                PAUSE = !PAUSE;
            else if (key == 'q')
                break;
        }

        r.sleep();
    }

    return 0;
}
