#include "rgbd/client.h"
#include "rgbd/image.h"
#include "rgbd/view.h"

#include <geolib/sensors/DepthCamera.h>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
    if (argc <= 1)
    {
        std::cout << "Please provide rgbd topic" << '\n';
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_viewer", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    rgbd::Client client;
    client.initialize(node->get_node_topics_interface()->resolve_topic_name(argv[1]));

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (!client.nextImage(image))
        {
            r.sleep();
            continue;
        }

        const cv::Mat& depth = image.getDepthImage();
        const cv::Mat& rgb = image.getRGBImage();

        std::cout << "------------------------------------------------" << '\n';
        std::cout << "time: " << image.getTimestamp() << '\n';

        if (depth.data)
        {
            const rgbd::View view(image, depth.cols);
            const geo::DepthCamera& cam_model = view.getRasterizer();

            std::cout << "depth:" << '\n';
            std::cout << "    camera model:" << '\n';
            std::cout << "        fx, fy = " << cam_model.getFocalLengthX() << ", " << cam_model.getFocalLengthY()
                      << '\n';
            std::cout << "        cx, cy = " << cam_model.getOpticalCenterX() << ", " << cam_model.getOpticalCenterY()
                      << '\n';
            std::cout << "        Tx, Ty = " << cam_model.getOpticalTranslationX() << ", "
                      << cam_model.getOpticalTranslationY() << '\n';
            std::cout << "    size = " << depth.cols << " x " << depth.rows << '\n';
        }
        else
        {
            std::cout << "depth: NO INFO" << '\n';
        }

        if (rgb.data)
        {
            std::cout << "rgb:" << '\n';
            std::cout << "    size = " << rgb.cols << " x " << rgb.rows << '\n';
        }
        else
        {
            std::cout << "rgb: NO INFO" << '\n';
        }

        r.sleep();
    }

    RCLCPP_INFO(logger, "Shutting down");
    rclcpp::shutdown();
    return 0;
}
