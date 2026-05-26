#include "rgbd/client.h"
#include "rgbd/image.h"
#include "rgbd/serialization.h"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_saver", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    rgbd::Client client;
    client.initialize("rgbd");

    rgbd::Image image;

    rclcpp::Rate r(rate);
    char key_pressed;
    while (rclcpp::ok())
    {
        RCLCPP_INFO(logger, "Press s to save and q to exit.");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
        system("/bin/stty raw");
        key_pressed = getchar();
        system("/bin/stty cooked");
#pragma GCC diagnostic pop

        if (key_pressed == 's')
        {
            if (client.nextImage(image))
            {
                std::stringstream ss;
                ss << "image_";
                double sec;
                const double fractional = std::modf(image.getTimestamp(), &sec);
                const std::time_t time = sec;
                ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H.%M.%S");
                ss << "." << std::setw(6) << std::setfill('0') << static_cast<int32_t>(fractional * 1e6);
                ss << ".rgbd";
                const std::string file_name = ss.str();

                std::ofstream f_out;
                f_out.open(file_name.c_str(), std::ifstream::binary);
                try
                {
                    tue::serialization::OutputArchive a_out(f_out);
                    rgbd::serialize(image, a_out);
                    RCLCPP_INFO_STREAM(logger, "Written image to '" << file_name << "'");
                }
                catch (const std::exception& e) // caught by reference to base
                {
                    RCLCPP_ERROR_STREAM(logger, "Error while writing to '" << file_name << "':\n" << e.what());
                }
                f_out.close();
            }
        }
        else if (key_pressed == 'q')
        {
            RCLCPP_INFO(logger, "Exiting");
            rclcpp::shutdown();
            return 0;
        }
        r.sleep();
    }

    RCLCPP_INFO(logger, "No image stored.");

    rclcpp::shutdown();
    return 0;
}
