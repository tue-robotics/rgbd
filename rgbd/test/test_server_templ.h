#ifndef TEST_SERVER_TEMPL_H_
#define TEST_SERVER_TEMPL_H_

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif

#include "rgbd/image.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#if __has_include(<sensor_msgs/msg/camera_info.hpp>)
#include <sensor_msgs/msg/camera_info.hpp>
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
#else
#include <sensor_msgs/CameraInfo.h>
using CameraInfoMsg = sensor_msgs::CameraInfo;
#endif
#if __has_include(<sensor_msgs/distortion_models.hpp>)
#include <sensor_msgs/distortion_models.hpp>
#else
#include <sensor_msgs/distortion_models.h>
#endif

/**
 * Template function to test the communication provide by a server.
 * It generates artificial images and sends these by the provide server.
 * The server should have a
 * @code
 * bool initialize(std::string servername)
 * @endcode
 * and a
 * @code
 * void send(const rgbd::Image& image)
 * @endcode
 * function.
 */
template <class T> int main_templ(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_transport_test_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    T server;
    server.initialize(node->get_node_topics_interface()->resolve_topic_name("test"));

    rclcpp::Rate r(rate);
    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat depth_image(480, 640, CV_32FC1, 5.0);
    CameraInfoMsg cam_info;
    cam_info.K = {554.2559327880068, 0.0, 320.5, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 1.0};
    cam_info.P = {554.2559327880068, 0.0, 320.5, 0.0, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = 640;
    cam_info.height = 480;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);

    rgbd::Image image(rgb_image, depth_image, cam_model, "test_frame_id", node->now().seconds());

    int x = 0;
    while (rclcpp::ok())
    {
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(0, 0, 255));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 5.0);
        x = (x + 10) % rgb_image.cols;
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(255, 0, 0));
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 1.0);

        image.setRGBImage(rgb_image);
        image.setDepthImage(depth_image);
        image.setTimestamp(node->now().seconds());

        server.send(image);

        r.sleep();
    }

    RCLCPP_INFO(logger, "Shutting down");
    rclcpp::shutdown();
    return 0;
}

#endif // TEST_SERVER_TEMPL_H_
