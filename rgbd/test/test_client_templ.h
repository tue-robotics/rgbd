#ifndef TEST_CLIENT_TEMPL_H_
#define TEST_CLIENT_TEMPL_H_

#include <opencv2/highgui/highgui.hpp>

#include "rgbd/image.h"

#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
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
template <class T> int main_templ(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_transport_test_client", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    bool headless = false;
    std::string arg;
    for (int i = 1; i < argc; ++i)
    {
        arg = argv[i];
        if (arg == "--headless")
        {
            headless = true;
            RCLCPP_INFO(logger, "Running in headless mode");
        }
        else if (arg == "--help")
        {
            usage();
            return 1;
        }
        else
        {
            RCLCPP_WARN_STREAM(logger, "Incorrect argument: '" << arg);
        }
    }

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    T client;
    if (!client.initialize(node->get_node_topics_interface()->resolve_topic_name("test")))
    {
        RCLCPP_FATAL(logger, "Could not initialize the client");
        return 1;
    }

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (client.nextImage(image))
        {
            std::cout << "Image: t = " << std::fixed << std::setprecision(12) << image.getTimestamp()
                      << ", frame = " << image.getFrameId() << std::endl;

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

    rclcpp::shutdown();
    return 0;
}

#endif // TEST_CLIENT_TEMPL_H_
