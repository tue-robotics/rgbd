#include "rgbd/server_ros.h"

#include <cstdint>
#include <geolib/datatypes.h>
#include <opencv2/core/matx.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp> // IWYU pragma: keep
#include <sensor_msgs/msg/image.hpp> // IWYU pragma: keep
#include <sensor_msgs/msg/point_cloud2.hpp> // IWYU pragma: keep
#include <string>

#include "rgbd/image.h"
#include "rgbd/ros/conversions.h"
#include "rgbd/view.h"

namespace rgbd
{

ServerROS::ServerROS(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_server_ros"))
{
}

ServerROS::~ServerROS() = default;

void ServerROS::initialize(std::string ns, bool publish_rgb, bool publish_depth, bool publish_pc)
{
    if (!ns.empty() && ns.back() != '/')
    {
        ns.push_back('/');
    }
    // NOLINTBEGIN(misc-include-cleaner) sensor_msgs/msg/{image,camera_info,point_cloud2}.hpp are included; the
    // detail struct headers suggested instead lack the type-support needed for create_publisher<T>.
    if (publish_rgb)
    {
        pub_rgb_img_ = node_->create_publisher<sensor_msgs::msg::Image>(ns + "rgb/image", 1);
        pub_rgb_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "rgb/camera_info", 1);
    }
    if (publish_depth)
    {
        pub_depth_img_ = node_->create_publisher<sensor_msgs::msg::Image>(ns + "depth/image", 1);
        pub_depth_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "depth/camera_info", 1);
    }
    if (publish_pc)
    {
        pub_depth_pc_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(ns + "depth/points", 1);
    }
    // NOLINTEND(misc-include-cleaner)
}

void ServerROS::send(const Image& image)
{
    if ((pub_depth_img_ || pub_depth_pc_) && image.getDepthImage().data)
    {
        rgbd::View const view(image, image.getDepthImage().cols);

        if (pub_depth_img_ && (pub_depth_img_->get_subscription_count() || pub_depth_info_->get_subscription_count()))
        {
            sensor_msgs::msg::Image msg;
            sensor_msgs::msg::CameraInfo info_msg;

            rgbd::convert(image.getDepthImage(), view.getRasterizer(), msg, info_msg);

            msg.header.stamp = rclcpp::Time(static_cast<int64_t>(image.getTimestamp() * 1e9));
            msg.header.frame_id = image.getFrameId();
            info_msg.header = msg.header;

            pub_depth_img_->publish(msg);
            pub_depth_info_->publish(info_msg);
        }
        if (pub_depth_pc_ && pub_depth_pc_->get_subscription_count())
        {
            pcl::PointCloud<pcl::PointXYZRGB> pc_msg;

            pc_msg.header.stamp = static_cast<uint64_t>(image.getTimestamp() * 1e6);
            pc_msg.header.frame_id = image.getFrameId();
            pc_msg.width = 0;
            pc_msg.height = 1;
            pc_msg.is_dense = true;

            for (int y = 0; y < view.getHeight(); ++y)
            {
                for (int x = 0; x < view.getWidth(); ++x)
                {
                    geo::Vector3 p;
                    if (view.getPoint3D(x, y, p))
                    {
                        pc_msg.points.emplace_back();
                        pcl::PointXYZRGB& p_pcl = pc_msg.points.back();
                        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access) pcl::PointXYZRGB exposes its
                        // fields via a union; there is no non-union accessor.
                        p_pcl.x = static_cast<float>(p.x);
                        p_pcl.y = static_cast<float>(-p.y);
                        p_pcl.z = static_cast<float>(-p.z);
                        const cv::Vec3b& c = view.getColor(x, y);
                        p_pcl.r = c[2];
                        p_pcl.g = c[1];
                        p_pcl.b = c[0];
                        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
                        ++pc_msg.width;
                    }
                    else
                    {
                        pc_msg.is_dense = false;
                    }
                }
            }

            sensor_msgs::msg::PointCloud2 pc2_msg;
            pcl::toROSMsg(pc_msg, pc2_msg);
            pc2_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(image.getTimestamp() * 1e9));
            pc2_msg.header.frame_id = image.getFrameId();
            pub_depth_pc_->publish(pc2_msg);
        }
    }

    if (pub_rgb_img_ && (pub_rgb_img_->get_subscription_count() || pub_rgb_info_->get_subscription_count()) &&
        image.getRGBImage().data)
    {
        rgbd::View const view(image, image.getRGBImage().cols);

        sensor_msgs::msg::Image msg;
        sensor_msgs::msg::CameraInfo info_msg;

        rgbd::convert(image.getRGBImage(), view.getRasterizer(), msg, info_msg);

        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(image.getTimestamp() * 1e9));
        msg.header.frame_id = image.getFrameId();
        info_msg.header = msg.header;

        pub_rgb_img_->publish(msg);
        pub_rgb_info_->publish(info_msg);
    }
}

} // namespace rgbd
