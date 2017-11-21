#include "rgbd/Client.h"
#include "rgbd/View.h"
//#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rgbd/ros/conversions.h>

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please specify rgbd topic / name" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "rgbd_to_ros");

    // Listener
    rgbd::Client client;
    client.intialize(argv[1]);

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
    ros::Publisher pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
    ros::Publisher pub_rgb_img = nh.advertise<sensor_msgs::Image>("rgb/image", 1);
    ros::Publisher pub_depth_img = nh.advertise<sensor_msgs::Image>("depth/image", 1);

    ros::Time last_image_stamp;

    ros::Rate r(30);
    while (ros::ok())
    {
        if (!last_image_stamp.isZero() && ros::Time::now() - last_image_stamp > ros::Duration(5.0))
        {
          ROS_ERROR("rgbd to ros did not receive images for 5 seconds ... restarting ...");
          exit(1);
        }

        rgbd::Image image;
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                last_image_stamp = ros::Time(image.getTimestamp());

                // Convert camera info to message
                rgbd::View view(image, image.getDepthImage().cols);

                // Convert to image messages
                sensor_msgs::Image msg;
                sensor_msgs::CameraInfo info_msg;
                
                rgbd::convert(image.getDepthImage(), view.getRasterizer(), msg, info_msg);

                msg.header.stamp = ros::Time(image.getTimestamp());
                msg.header.frame_id = image.getFrameId();
                info_msg.header = msg.header;   

                // Publish
                pub_depth_img.publish(msg);
                pub_depth_info.publish(info_msg);
            }

            if (image.getRGBImage().data)
            {
                // Convert camera info to message
                rgbd::View view(image, image.getRGBImage().cols);

                // Convert to image messages
                sensor_msgs::Image msg;
                sensor_msgs::CameraInfo info_msg;
                
                rgbd::convert(image.getRGBImage(), view.getRasterizer(), msg, info_msg);

                msg.header.stamp = ros::Time(image.getTimestamp());
                msg.header.frame_id = image.getFrameId();
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
