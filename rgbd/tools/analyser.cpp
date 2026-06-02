#include "rgbd/client.h"
#include "rgbd/view.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    if (argc <= 1)
    {
        std::cout << "Please provide rgbd topic" << std::endl;
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

        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "time: " << image.getTimestamp() << std::endl;

        if (depth.data)
        {
            rgbd::View view(image, depth.cols);
            const geo::DepthCamera& cam_model = view.getRasterizer();

            std::cout << "depth:" << std::endl;
            std::cout << "    camera model:" << std::endl;
            std::cout << "        fx, fy = " << cam_model.getFocalLengthX() << ", " << cam_model.getFocalLengthY()
                      << std::endl;
            std::cout << "        cx, cy = " << cam_model.getOpticalCenterX() << ", " << cam_model.getOpticalCenterY()
                      << std::endl;
            std::cout << "        Tx, Ty = " << cam_model.getOpticalTranslationX() << ", "
                      << cam_model.getOpticalTranslationY() << std::endl;
            std::cout << "    size = " << depth.cols << " x " << depth.rows << std::endl;
        }
        else
        {
            std::cout << "depth: NO INFO" << std::endl;
        }

        if (rgb.data)
        {
            std::cout << "rgb:" << std::endl;
            std::cout << "    size = " << rgb.cols << " x " << rgb.rows << std::endl;
        }
        else
        {
            std::cout << "rgb: NO INFO" << std::endl;
        }

        r.sleep();
    }

    RCLCPP_INFO(logger, "Shutting down");
    rclcpp::shutdown();
    return 0;
}
