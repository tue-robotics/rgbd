#include "rgbd/client.h"
#include "rgbd/ros_compat.h"

// Writing video files
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_transport_server");

    // Read parameters
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    std::string filename;
    nh_private.getParam("filename", filename);

    std::string format = "DIVX";
    nh_private.getParam("format", format);

    double size = 1;
    nh_private.getParam("size", size);

    if (format.size() != 4)
    {
        ROS_ERROR("Parameter 'format' should be string of size 4 (e.g., MJPG, "
                  "DIVX, MPG4, etc)");
        return 1;
    }

    rgbd::Client client;
    if (!client.initialize("rgbd"))
    {
        ROS_ERROR("Could not initialize rgbd client");
        return 1;
    }

    cv::VideoWriter video_writer;
    bool initialized = false;

    // video size
    cv::Size2i video_size;

    ros::WallTime last_master_check = ros::WallTime::now();

    // Start loop at given frequency
    ros::Rate r(rate);
    rgbd::Image image;
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

    return 0;
}
