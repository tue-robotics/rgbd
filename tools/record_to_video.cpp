// ROS
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

// ROS image message
#include <sensor_msgs/Image.h>

// For converting image messages to OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Writing video files
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

// Received RGB image
cv::Mat rgb_image;

// ----------------------------------------------------------------------------------------

void imageCallback(const sensor_msgs::ImageConstPtr& rgb_image_msg) {

    cv_bridge::CvImagePtr img_ptr;

    // Convert RGB image message to OpenCV mat
    try {
        img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not deserialize rgb image: %s", e.what());
        return;
    }

    rgb_image = img_ptr->image;
}

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_server");

    // Read parameters
    ros::NodeHandle nh_private("~");

    double video_frame_rate = 30;
    nh_private.getParam("fps", video_frame_rate);

    std::string filename;
    nh_private.getParam("filename", filename);

    std::string format = "DIVX";
    nh_private.getParam("format", format);

    double size = 1;
    nh_private.getParam("size", size);

    if (format.size() != 4)
    {
        ROS_ERROR("Parameter 'format' should be string of size 4 (e.g., MJPG, DIVX, MPG4, etc)");
        return 1;
    }

    // Subscriber to image topic
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("rgb", 1, imageCallback);

    cv::VideoWriter video_writer;
    bool initialized = false;

    // video size
    cv::Size2i video_size;

    // Start loop at given frequency
    ros::Rate r(video_frame_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        // Check if we already received an image
        if (rgb_image.data)
        {
            // Check ik we already initialized the video writer
            if (!initialized)
            {
                // If not, do so
                video_size = cv::Size2i(size * rgb_image.cols, size * rgb_image.rows);
                video_writer.open(filename.c_str(), CV_FOURCC(format[0], format[1], format[2], format[3]), video_frame_rate, video_size);

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
