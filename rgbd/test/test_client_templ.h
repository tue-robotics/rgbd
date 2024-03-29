#ifndef TEST_CLIENT_TEMPL_H_
#define TEST_CLIENT_TEMPL_H_

#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include "rgbd/image.h"

#include <string>


void usage()
{
    std::cout << "Usage: rgbd_test_client_TYPE [--headless] [--help]" << std::endl;
}

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

    bool headless = false;
    std::string arg;
    for (int i=1; i<argc; ++i)
    {
        arg = argv[i];
        if (arg == "--headless")
        {
            headless = true;
            ROS_INFO("Running in headless mode");
        }
        else if(arg == "--help")
        {
            usage();
            return 1;
        }
        else
        {
            ROS_WARN_STREAM("Incorrect argument: '" << arg);
        }
    }

    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    T client;
    if (!client.initialize(ros::names::resolve("test")))
    {
        ROS_FATAL("Could not initialize the client");
        return 1;
    }

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
            std::cout << "Image: t = " << std::fixed << std::setprecision(12) << image.getTimestamp() << ", frame = " << image.getFrameId() << std::endl;

            if (!headless)
            {
                cv::imshow("rgb", image.getRGBImage());
                cv::imshow("depth", image.getDepthImage() / 8);
                cv::waitKey(3);
            }
        }
        r.sleep();
    }

    if (!headless)
    {
        cv::destroyAllWindows();
    }

    return 0;
}

#endif // TEST_CLIENT_TEMPL_H_
