#include "rgbd/client.h"
#include "rgbd/image.h"

// Writing video files
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_transport_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    // Read parameters
    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    std::string filename;
    if (!node->has_parameter("filename"))
    {
        node->declare_parameter<std::string>("filename", filename);
    }
    node->get_parameter("filename", filename);

    std::string format = "DIVX";
    if (!node->has_parameter("format"))
    {
        node->declare_parameter<std::string>("format", format);
    }
    node->get_parameter("format", format);

    double size = 1;
    if (!node->has_parameter("size"))
    {
        node->declare_parameter<double>("size", size);
    }
    node->get_parameter("size", size);

    if (format.size() != 4)
    {
        RCLCPP_ERROR(logger, "Parameter 'format' should be string of size 4 (e.g., MJPG, DIVX, MPG4, etc)");
        return 1;
    }

    rgbd::Client client;
    if (!client.initialize(node->get_node_topics_interface()->resolve_topic_name("rgbd")))
    {
        RCLCPP_ERROR(logger, "Could not initialize rgbd client");
        return 1;
    }

    cv::VideoWriter video_writer;
    bool initialized = false;

    // video size
    cv::Size2i video_size;

    // Start loop at given frequency
    rclcpp::Rate r(rate);
    rgbd::Image image;
    while (rclcpp::ok())
    {
        if (!client.nextImage(image) || !image.getRGBImage().data)
        {
            r.sleep();
            continue;
        }
        const cv::Mat& rgb_image = image.getRGBImage();
        if (rgb_image.data)
        {
            // Check ik we already initialized the video writer
            if (!initialized)
            {
                // If not, do so
                video_size =
                    cv::Size2i(static_cast<int>(size * rgb_image.cols), static_cast<int>(size * rgb_image.rows));
                video_writer.open(filename.c_str(),
                                  cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]),
                                  rate,
                                  video_size);

                if (!video_writer.isOpened())
                {
                    // Could not create the video writer, so exit
                    std::cout << "Unable to create video writer" << std::endl;
                    return 1;
                }

                initialized = true;
            }

            if (size == 1)
            {
                // Write received image to the video
                video_writer.write(rgb_image);
            }
            else
            {
                cv::Mat rgb_image_scaled;
                cv::resize(rgb_image, rgb_image_scaled, video_size);

                // Write received image to the video
                video_writer.write(rgb_image_scaled);
            }
        }

        // Sleep for remaining loop time
        r.sleep();
    }

    video_writer.release();

    rclcpp::shutdown();
    return 0;
}
