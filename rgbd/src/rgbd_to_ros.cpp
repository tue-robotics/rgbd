#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <ros/init.h>
#include <ros/console.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <ros/rate.h>

#include "rgbd/client.h"
#include "rgbd/view.h"
#include <rgbd/ros/conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char **argv)
{
    bool publish_rgb = false, publish_depth = false, publish_pc = false;
    {
        bool valid_arg_provided = false;
        for (int i = 1; i < argc; ++i)
        {
            std::string opt(argv[i]);
            if (opt == "-h" || opt == "--help")
            {
                std::cout << "Usage: rgbd_to_ros [OPTIONS]" << std::endl
                          << "    If no valid options are provided, rgb and depth images and camera info will be published" << std::endl
                          << "Options:" << std::endl
                          << "    -h, --help:         show this message" << std::endl
                          << "    -a, --all:          publish rgb, depth and pointcloud" << std::endl
                          << "    --rgb, --color:     publish rgb image and camera info" << std::endl
                          << "    --depth:            publish depth image and camera info" << std::endl
                          << "    --rgbd:             publish rgb and depth images and camera info" << std::endl
                          << "    --pc, --pointcloud: publish pointcloud" << std::endl;
                return 0;
            }
            else if (opt == "-a" || opt == "--all")
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
        }
        if (!valid_arg_provided)
        {
            publish_rgb = true;
            publish_depth = true;
        }
    }

    ROS_DEBUG_STREAM("publish_rgb: " << publish_rgb);
    ROS_DEBUG_STREAM("publish_depth: " << publish_depth);
    ROS_DEBUG_STREAM("publish_pc: " << publish_pc);

    ros::init(argc, argv, "rgbd_to_ros");
    ros::start(); // Required to use ros::names::resolve, without creating a nodehandle

    // Listener
    rgbd::Client client;
    client.intialize(ros::names::resolve("rgbd"));

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_rgb_img, pub_rgb_info, pub_depth_img, pub_depth_info, pub_depth_pc;
    if (publish_rgb)
    {
        pub_rgb_img = nh.advertise<sensor_msgs::Image>("rgb/image", 1);
        pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
    }
    if (publish_depth)
    {
        pub_depth_img = nh.advertise<sensor_msgs::Image>("depth/image", 1);
        pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
    }
    if (publish_pc)
    {
        pub_depth_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("depth/points", 1);
    }

    ROS_DEBUG_STREAM("rgb image topic: " << pub_rgb_img.getTopic());
    ROS_DEBUG_STREAM("rgb camera info topic: " << pub_rgb_info.getTopic());
    ROS_DEBUG_STREAM("depth image topic: " << pub_depth_img.getTopic());
    ROS_DEBUG_STREAM("depth camera info topic: " << pub_depth_info.getTopic());
    ROS_DEBUG_STREAM("pointcloud topic: " << pub_depth_pc.getTopic());

    ros::Time last_image_stamp;

    ros::Rate r(30);
    while (ros::ok() && ros::master::check())
    {
        if (!last_image_stamp.isZero() && ros::Time::now() - last_image_stamp > ros::Duration(5.0))
        {
          ROS_ERROR("rgbd to ros did not receive images for 5 seconds ... restarting ...");
          exit(1);
        }

        rgbd::Image image;
        if (client.nextImage(image))
        {
            if ((publish_depth || publish_pc) && image.getDepthImage().data)
            {
                last_image_stamp = ros::Time(image.getTimestamp());

                // Convert camera info to message
                rgbd::View view(image, image.getDepthImage().cols);


                if (publish_depth)
                {
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
                if (publish_pc)
                {
                     // Create point cloud
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_msg(new pcl::PointCloud<pcl::PointXYZRGB>());

                     pc_msg->header.stamp = static_cast<uint64_t>(image.getTimestamp() * 1e6);
                     pc_msg->header.frame_id = image.getFrameId();
                     pc_msg->width  = 0;
                     pc_msg->height  = 1;
                     pc_msg->is_dense = true;

                     // Fill point cloud
                     for(int y = 0; y < view.getHeight(); ++y)
                     {
                         for(int x = 0; x < view.getWidth(); ++x)
                         {
                             geo::Vector3 p;
                             if (view.getPoint3D(x, y, p))
                             {
                                 const cv::Vec3b& c = view.getColor(x, y);

                                 // Push back and correct for geolib frame
                                 pc_msg->points.push_back(pcl::PointXYZRGB());
                                 pcl::PointXYZRGB& p_pcl = pc_msg->points.back();
                                 p_pcl.x = static_cast<float>(p.x);
                                 p_pcl.y = static_cast<float>(-p.y);
                                 p_pcl.z = static_cast<float>(-p.z);
                                 p_pcl.r = c[2];
                                 p_pcl.g = c[1];
                                 p_pcl.b = c[0];
                                 pc_msg->width++;
                             }
                             else
                             {
                                 pc_msg->is_dense = false;
                             }
                         }
                     }

                     // Publish
                     pub_depth_pc.publish(pc_msg);
                }
            }

            if (publish_rgb && image.getRGBImage().data)
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
