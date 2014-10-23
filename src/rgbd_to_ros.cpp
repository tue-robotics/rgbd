#include "rgbd/Client.h"
#include "rgbd/View.h"
//#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rgbd/ros/conversions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_to_ros");

    // Listener
    rgbd::Client client;
    client.intialize("rgbd");

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("/rgb/camera_info", 1);
    ros::Publisher pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("/depth/camera_info", 1);
    ros::Publisher pub_rgb_img = nh.advertise<sensor_msgs::Image>("/rgb/image", 1);
    ros::Publisher pub_depth_img = nh.advertise<sensor_msgs::Image>("/depth/image", 1);

    ros::Rate r(30);
    while (ros::ok())
    {
        rgbd::Image image;
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                // Convert image to message
                sensor_msgs::Image msg;
                rgbd::convert(image.getDepthImage(), msg);
                msg.header.stamp = ros::Time(image.getTimestamp());
                msg.header.frame_id = image.getFrameId();

                // Convert camera info to message
                rgbd::View view(image, image.getDepthImage().cols);
                sensor_msgs::CameraInfo info_msg;
                rgbd::convert(view.getRasterizer(), info_msg);
                info_msg.header = msg.header;

                // Publish
                pub_depth_img.publish(msg);
                pub_depth_info.publish(info_msg);
            }

            if (image.getRGBImage().data)
            {
                // Convert image to message
                sensor_msgs::Image msg;
                rgbd::convert(image.getRGBImage(), msg);
                msg.header.stamp = ros::Time(image.getTimestamp());
                msg.header.frame_id = image.getFrameId();

                // Convert camera info to message
                rgbd::View view(image, image.getRGBImage().cols);
                sensor_msgs::CameraInfo info_msg;
                rgbd::convert(view.getRasterizer(), info_msg);
                info_msg.header = msg.header;

                // Publish
                pub_rgb_img.publish(msg);
                pub_rgb_info.publish(info_msg);
            }
        }

        r.sleep();
    }

    return 0;
}
