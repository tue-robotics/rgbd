#include "rgbd/image.h"
#include <algorithm>
#include <boost/circular_buffer.hpp> // IWYU pragma: keep
#include <cmath>
#include <cstdint>
#include <geolib/math_types.h>
#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <rgbd/client.h>
#include <rgbd/view.h>

#include <rgbd_interfaces/srv/project2_d_to3_d.hpp> // IWYU pragma: keep

#include <memory>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/detail/region_of_interest__struct.hpp>
#include <utility>
#include <vector>

namespace
{

template <typename NodeT, typename CallbackT>
auto createProject2DTo3DService(NodeT& node,
                                CallbackT&& callback,
                                const rclcpp::CallbackGroup::SharedPtr& callback_group,
                                int)
    // rgbd_interfaces/srv/project2_d_to3_d.hpp is included; the detail struct header suggested instead lacks the
    // type-support needed for create_service<T>.
    // NOLINTNEXTLINE(misc-include-cleaner)
    -> decltype(node->template create_service<rgbd_interfaces::srv::Project2DTo3D>(
        "project_2d_to_3d",
        std::forward<CallbackT>(callback),
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default),
                    rmw_qos_profile_services_default),
        callback_group))
{
    return node->template create_service<rgbd_interfaces::srv::Project2DTo3D>(
        "project_2d_to_3d",
        std::forward<CallbackT>(callback),
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default),
                    rmw_qos_profile_services_default),
        callback_group);
}

template <typename NodeT, typename CallbackT>
auto createProject2DTo3DService(NodeT& node,
                                CallbackT&& callback,
                                const rclcpp::CallbackGroup::SharedPtr& callback_group,
                                int64_t)
    -> decltype(node->template create_service<rgbd_interfaces::srv::Project2DTo3D>(
        "project_2d_to_3d", std::forward<CallbackT>(callback), rmw_qos_profile_services_default, callback_group))
{
    return node->template create_service<rgbd_interfaces::srv::Project2DTo3D>(
        "project_2d_to_3d", std::forward<CallbackT>(callback), rmw_qos_profile_services_default, callback_group);
}

// boost/circular_buffer.hpp is included; the detail base header suggested instead does not compile on its own.
// NOLINTNEXTLINE(misc-include-cleaner)
void srvGet3dPointFromROI(const boost::circular_buffer<std::shared_ptr<rgbd::Image>>& last_images,
                          const std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Request>& REQ,
                          const std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Response>& res)
{
    std::shared_ptr<rgbd::Image> last_image;

    if (!last_images.empty())
    {
        if (rclcpp::Time(REQ->stamp).nanoseconds() == 0)
            last_image = last_images.back();
        else
        {
            for (auto it = last_images.rbegin(); it != last_images.rend(); ++it)
            {
                if ((*it)->getTimestamp() <= rclcpp::Time(REQ->stamp).seconds())
                    last_image = *it;
            }
        }
    }

    if (!last_image)
    {
        return;
    }

    for (const sensor_msgs::msg::RegionOfInterest& roi : REQ->rois)
    {
        const cv::Mat& depth = last_image->getDepthImage();

        cv::Rect const roi_rgb(static_cast<int>(roi.x_offset),
                               static_cast<int>(roi.y_offset),
                               static_cast<int>(roi.width),
                               static_cast<int>(roi.height));
        float const rgb_depth_width_ratio =
            static_cast<float>(depth.cols) / static_cast<float>(last_image->getRGBImage().cols);
        cv::Rect const roi_depth(rgb_depth_width_ratio * roi_rgb.tl(), rgb_depth_width_ratio * roi_rgb.br());
        cv::Point const roi_depth_center = 0.5 * (roi_depth.tl() + roi_depth.br());

        cv::Rect const roi_depth_capped(
            cv::Point(std::max(0, roi_depth.x), std::max(0, roi_depth.y)),
            cv::Point(std::min(depth.cols - 1, roi_depth.br().x), std::min(depth.rows - 1, roi_depth.br().y)));

        cv::Mat depth_roi_capped = depth(roi_depth_capped);

        std::vector<float> depths;
        for (int j = 0; j < depth_roi_capped.cols * depth_roi_capped.rows; ++j)
        {
            float const d = depth_roi_capped.at<float>(j);
            if (d > 0 && d == d)
                depths.push_back(d);
        }

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = last_image->getFrameId();
        point_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(last_image->getTimestamp() * 1e9));
        if (depths.empty())
        {
            point_msg.point.x = point_msg.point.y = point_msg.point.z = static_cast<double>(NAN);
        }
        else
        {
            std::sort(depths.begin(), depths.end());
            float const median_depth = depths[depths.size() / 2];

            rgbd::View const view(*last_image, last_image->getDepthImage().cols);
            geo::Vec3 pos = view.getRasterizer().project2Dto3D(roi_depth_center.x, roi_depth_center.y) *
                            static_cast<double>(median_depth);
            pos.y = -pos.y;
            pos.z = -pos.z;
            geo::convert(pos, point_msg.point);
        }

        res->points.push_back(point_msg);
    }
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("get_3d_point_from_image_roi");

    double const rate = node->declare_parameter<double>("rate", 30.0);

    rgbd::Client client(node);
    client.initialize("rgbd");

    boost::circular_buffer<std::shared_ptr<rgbd::Image>> last_images(100);

    auto cb_group_srv = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto srv_project_2d_to_3d = createProject2DTo3DService(
        node,
        [&last_images](const std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Request>& req,
                       const std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Response>& res)
        { srvGet3dPointFromROI(last_images, req, res); },
        cb_group_srv,
        0);

    (void)srv_project_2d_to_3d;

    rgbd::Image image;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(cb_group_srv, node->get_node_base_interface());

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                last_images.push_back(std::make_shared<rgbd::Image>(image));
            }
        }
        executor.spin_some();
        r.sleep();
    }

    executor.remove_callback_group(cb_group_srv);
    rclcpp::shutdown();
    return 0;
}
