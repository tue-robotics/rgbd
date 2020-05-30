#include "rgbd/server_ros.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "rgbd/view.h"
#include "rgbd/ros/conversions.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


namespace rgbd {

// ----------------------------------------------------------------------------------------

ServerROS::ServerROS()
{
}

// ----------------------------------------------------------------------------------------

ServerROS::~ServerROS()
{
}

// ----------------------------------------------------------------------------------------

void ServerROS::initialize(std::string ns, const bool publish_rgb, const bool publish_depth, const bool publish_pc)
{
    if (!ns.empty() && ns.back() != '/')
    {
        ns.push_back('/');
    }
    if (publish_rgb)
    {
        pub_rgb_img_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::Image>(ns + "rgb/image", 1));
        pub_rgb_info_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::CameraInfo>(ns + "rgb/camera_info", 1));
        ROS_DEBUG_STREAM("rgb image topic: " << pub_rgb_img_->getTopic());
        ROS_DEBUG_STREAM("rgb camera info topic: " << pub_rgb_info_->getTopic());
    }
    else
    {
        ROS_DEBUG("rgb image publisher not initialized");
        ROS_DEBUG("rgb camera info publisher not initialized");
    }
    if (publish_depth)
    {
        pub_depth_img_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::Image>(ns + "depth/image", 1));
        pub_depth_info_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::CameraInfo>(ns + "depth/camera_info", 1));
        ROS_DEBUG_STREAM("depth image topic: " << pub_depth_img_->getTopic());
        ROS_DEBUG_STREAM("depth camera info topic: " << pub_depth_info_->getTopic());
    }
    else
    {
        ROS_DEBUG("depth image publisher not initialized");
        ROS_DEBUG("depth camera info publisher not initialized");
    }
    if (publish_pc)
    {
        pub_depth_pc_ = std::make_shared<ros::Publisher>(nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(ns + "depth/points", 1));
        ROS_DEBUG_STREAM("pointcloud topic: " << pub_depth_pc_->getTopic());
    }
    else
    {
        ROS_DEBUG("pointcloud publisher not initialized");
    }

}

// ----------------------------------------------------------------------------------------

void ServerROS::send(const Image& image)
{
    if ((pub_depth_img_ || pub_depth_pc_) && image.getDepthImage().data)
    {
        // Convert camera info to message
        rgbd::View view(image, image.getDepthImage().cols);


        if (pub_depth_img_ && (pub_depth_img_->getNumSubscribers() || pub_depth_img_->getNumSubscribers()))
        {
            // Convert to image messages
            sensor_msgs::Image msg;
            sensor_msgs::CameraInfo info_msg;

            rgbd::convert(image.getDepthImage(), view.getRasterizer(), msg, info_msg);

            msg.header.stamp = ros::Time(image.getTimestamp());
            msg.header.frame_id = image.getFrameId();
            info_msg.header = msg.header;

            // Publish
            pub_depth_img_->publish(msg);
            pub_depth_info_->publish(info_msg);
        }
        if (pub_depth_pc_ && pub_depth_pc_->getNumSubscribers())
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
             pub_depth_pc_->publish(pc_msg);
        }
    }

    if (pub_rgb_img_ && (pub_rgb_img_->getNumSubscribers() || pub_rgb_info_->getNumSubscribers()) && image.getRGBImage().data)
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
        pub_rgb_img_->publish(msg);
        pub_rgb_info_->publish(info_msg);
    }
}

}
