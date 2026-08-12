#include "rgbd/client.h"
#include "rgbd/image.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <memory>
#include <opencv2/imgproc.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_viewer", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    rgbd::Client client;
    client.initialize(node->get_node_topics_interface()->resolve_topic_name("rgbd"));

    const std::string window_name = "RGBD_VIEW";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    bool pause = false;

    cv::Mat canvas;

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (!pause && client.nextImage(image))
        {
            // Show rgb image
            if (image.getRGBImage().data)
                canvas = image.getRGBImage();
        }

        if (pause)
            cv::putText(canvas,
                        "PAUSED",
                        cv::Point(10, canvas.rows - 25),
                        cv::FONT_HERSHEY_COMPLEX_SMALL,
                        1,
                        cv::Scalar(255, 255, 255),
                        1);

        cv::imshow(window_name, canvas);

        const int i_key = cv::waitKey(3);
        if (i_key >= 0)
        {
            const char key = static_cast<char>(i_key);

            if (key == ' ')
                pause = !pause;
            else if (key == 'q')
                break;
        }

        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
