#include <cstddef>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "rgbd/client.h"
#include "rgbd/image.h"
#include "rgbd/server_ros.h"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    std::vector<std::string> myargv = rclcpp::remove_ros_arguments(argc, argv);
    bool publish_rgb = false;
    bool publish_depth = false;
    bool publish_pc = false;
    {
        bool valid_arg_provided = false;
        for (size_t i = 1; i < myargv.size(); ++i)
        {
            const std::string& opt = myargv[i];
            if (opt == "-h" || opt == "--help")
            {
                std::cout
                    << "Usage: rgbd_to_ros [OPTIONS]" << '\n'
                    << "    If no valid options are provided, rgb and depth images and camera info will be published"
                    << '\n'
                    << "Options:" << '\n'
                    << "    -h, --help:         show this message" << '\n'
                    << "    -a, --all:          publish rgb, depth and pointcloud" << '\n'
                    << "    --rgb, --color:     publish rgb image and camera info" << '\n'
                    << "    --depth:            publish depth image and camera info" << '\n'
                    << "    --rgbd:             publish rgb and depth images and camera info" << '\n'
                    << "    --pc, --pointcloud: publish pointcloud" << '\n';
                return 0;
            }
            if (opt == "-a" || opt == "--all")
            {
                publish_rgb = true;
                publish_depth = true;
                publish_pc = true;
                valid_arg_provided = true;
            }
            else if (opt == "--rgb" || opt == "--color")
            {
                publish_rgb = true;
                valid_arg_provided = true;
            }
            else if (opt == "--depth")
            {
                publish_depth = true;
                valid_arg_provided = true;
            }
            else if (opt == "--rgbd")
            {
                publish_rgb = true;
                publish_depth = true;
                valid_arg_provided = true;
            }
            else if (opt == "--pc" || opt == "--pointcloud")
            {
                publish_pc = true;
                valid_arg_provided = true;
            }
            else if (!opt.compare(0, 2, "--"))
            {
                std::cout << "[rgbd_to_ros] Unknown option: '" << opt << "'." << '\n';
                return 1;
            }
            else
            {
                std::cout << "[rgbd_to_ros] Ignoring option: '" << opt << "'." << '\n';
            }
        }
        if (!valid_arg_provided)
        {
            publish_rgb = true;
            publish_depth = true;
        }
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgbd_to_ros");

    RCLCPP_DEBUG(node->get_logger(), "publish_rgb: %d", publish_rgb);
    RCLCPP_DEBUG(node->get_logger(), "publish_depth: %d", publish_depth);
    RCLCPP_DEBUG(node->get_logger(), "publish_pc: %d", publish_pc);

    rgbd::Client client(node);
    client.initialize("rgbd");

    rgbd::ServerROS server(node);
    server.initialize("", publish_rgb, publish_depth, publish_pc);

    double const rate = node->declare_parameter<double>("rate", 30.0);

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (client.nextImage(image))
        {
            server.send(image);
        }

        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
