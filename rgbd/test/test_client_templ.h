#ifndef TEST_CLIENT_TEMPL_H_
#define TEST_CLIENT_TEMPL_H_

#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>

#include "rgbd/image.h"

/**
 * Template function to test the communication of a client class.
 * The client should have a
 * @code
 * bool initialize(std::string servername)
 * @endcode
 * and a
 * @code
 * bool nextImage(rgbd::Image& image)
 * @endcode
 * function.
 * Both the RGB and depth image are shown in seperate windows.
 */
template<class T>
int main_templ(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_client");
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    T client;
    client.initialize(ros::names::resolve("test"));

    rgbd::Image image;

    ros::Rate r(rate);
    while (ros::ok())
    {
        if (!ros::master::check())
        {
            ROS_ERROR("Lost connection to master");
            return 1;
        }
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

#endif // TEST_CLIENT_TEMPL_H_
