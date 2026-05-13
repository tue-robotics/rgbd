#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>
#include <algorithm>
#include <cmath>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <rgbd/client.h>
#include <rgbd/view.h>

#include <rgbd_interfaces/srv/project2_d_to3_d.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

#include <boost/circular_buffer.hpp>

boost::circular_buffer<std::shared_ptr<rgbd::Image>> g_last_images_;

void srvGet3dPointFromROI(const std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Request> req,
                          std::shared_ptr<rgbd_interfaces::srv::Project2DTo3D::Response> res)
{
    std::shared_ptr<rgbd::Image> last_image;

    if (!g_last_images_.empty())
    {
        if (rclcpp::Time(req->stamp).nanoseconds() == 0)
            last_image = g_last_images_.back();
        else
        {
            for (auto it = g_last_images_.rbegin(); it != g_last_images_.rend(); ++it)
            {
                if ((*it)->getTimestamp() <= rclcpp::Time(req->stamp).seconds())
                    last_image = *it;
            }
        }
    }

    if (!last_image)
    {
        return;
    }

    for (const sensor_msgs::msg::RegionOfInterest& roi : req->rois)
    {
        const cv::Mat& depth = last_image->getDepthImage();

        cv::Rect roi_rgb(static_cast<int>(roi.x_offset), static_cast<int>(roi.y_offset),
                         static_cast<int>(roi.width), static_cast<int>(roi.height));
        float rgb_depth_width_ratio = static_cast<float>(depth.cols) / static_cast<float>(last_image->getRGBImage().cols);
        cv::Rect roi_depth(rgb_depth_width_ratio * roi_rgb.tl(), rgb_depth_width_ratio * roi_rgb.br());
        cv::Point roi_depth_center = 0.5 * (roi_depth.tl() + roi_depth.br());

        cv::Rect roi_depth_capped(cv::Point(std::max(0, roi_depth.x),
                                            std::max(0, roi_depth.y)),
                                  cv::Point(std::min(depth.cols - 1, roi_depth.br().x),
                                            std::min(depth.rows - 1, roi_depth.br().y)));

        cv::Mat depth_roi_capped = depth(roi_depth_capped);

        std::vector<float> depths;
        for (int j = 0; j < depth_roi_capped.cols * depth_roi_capped.rows; ++j)
        {
            float d = depth_roi_capped.at<float>(j);
            if (d > 0 && d == d)
                depths.push_back(d);
        }

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = last_image->getFrameId();
        point_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(last_image->getTimestamp() * 1e9)).to_msg();
        if (depths.empty())
        {
            point_msg.point.x = point_msg.point.y = point_msg.point.z = static_cast<double>(NAN);
        }
        else
        {
            std::sort(depths.begin(), depths.end());
            float median_depth = depths[depths.size() / 2];

            rgbd::View view(*last_image, last_image->getDepthImage().cols);
            geo::Vec3 pos = view.getRasterizer().project2Dto3D(roi_depth_center.x, roi_depth_center.y) * static_cast<double>(median_depth);
            pos.y = -pos.y;
            pos.z = -pos.z;
            geo::convert(pos, point_msg.point);
        }

        res->points.push_back(point_msg);
    }

    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("get_3d_point_from_image_roi");

    double rate = node->declare_parameter<double>("rate", 30.0);

    rgbd::Client client(node);
    client.initialize("rgbd");

    g_last_images_.set_capacity(100);

    auto srv_project_2d_to_3d = node->create_service<rgbd_interfaces::srv::Project2DTo3D>(
        "project_2d_to_3d", &srvGet3dPointFromROI);

    (void)srv_project_2d_to_3d;

    rgbd::Image image;

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                g_last_images_.push_back(std::make_shared<rgbd::Image>(image));
            }
        }
        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
