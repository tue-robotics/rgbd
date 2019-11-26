#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");
    ros::start();

    rgbd::Client client;
    client.intialize(ros::names::resolve("rgbd"));

    cv::namedWindow("RGBD VIEW", cv::WINDOW_NORMAL);
    cv::setWindowProperty("RGBD VIEW", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    bool PAUSE = false;

    cv::Mat canvas;

    ros::Rate r(30);
    while (ros::ok())
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

        cv::imshow("RGBD VIEW", canvas);

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
