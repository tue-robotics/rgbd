#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>

#include "rgbd/client.h"
#include "rgbd/view.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");
    ros::start();

    rgbd::Client client;
    client.intialize(ros::names::resolve("rgbd"));

    const std::string  window_name = "RGBD_VIEW";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    bool PAUSE = false;

    cv::Mat canvas;

    ros::Rate r(30);
    while (ros::ok() && ros::master::check())
    {
        rgbd::Image image;
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

    usleep(500000);

    return 0;
}
