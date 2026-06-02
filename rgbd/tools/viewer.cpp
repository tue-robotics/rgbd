#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rgbd/client.h"
#include "rgbd/view.h"

#include <memory>

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

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
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

                for (int y = 0; y < view.getHeight(); ++y)
                {
                    for (int x = 0; x < view.getWidth(); ++x)
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

    rclcpp::shutdown();
    return 0;
}
