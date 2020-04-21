#include <rgbd/view.h>
#include <rgbd/client.h>

#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <ros/init.h>
#include <ros/names.h>
#include <ros/service_client.h>
#include <ros/node_handle.h>
#include <ros/console.h>
#include <ros/time.h>

#include <sensor_msgs/RegionOfInterest.h>
#include <rgbd_msgs/Project2DTo3D.h>

#include <memory>

#include <boost/circular_buffer.hpp>

// ----------------------------------------------------------------------------------------------------

boost::circular_buffer<std::shared_ptr<rgbd::Image> > g_last_images_;
bool srvGet3dPointFromROI(rgbd_msgs::Project2DTo3D::Request& req, rgbd_msgs::Project2DTo3D::Response& res)
{
    std::shared_ptr<rgbd::Image> last_image;

    if (!g_last_images_.empty())
    {
        if (req.stamp == ros::Time(0))
            last_image = g_last_images_.back();
        else
        {
            for (auto it = g_last_images_.rbegin(); it != g_last_images_.rend(); ++it)
            {
                if ( (*it)->getTimestamp() <= req.stamp.toSec())
                    last_image = *it;
            }
        }
    }

    if (!last_image)
    {
        ROS_ERROR("I could not find an images in my image buffer for timestamp %.2f", req.stamp.toSec());
        return false;
    }

    for (const sensor_msgs::RegionOfInterest& roi : req.rois)
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

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Get median depth of ROI

        std::vector<float> depths;
        for (int j = 0; j < depth_roi_capped.cols * depth_roi_capped.rows; ++j)
        {
            float d = depth_roi_capped.at<float>(j);
            if (d > 0 && d == d) // filters NaN
                depths.push_back(d);
        }

        geometry_msgs::PointStamped point_msg;
        point_msg.header.frame_id = last_image->getFrameId();
        point_msg.header.stamp = ros::Time(last_image->getTimestamp());
        if (depths.empty())
        {
            ROS_ERROR("All depths within ROI are invalid! We will send a NAN point");
            point_msg.point.x = point_msg.point.y = point_msg.point.z = static_cast<double>(NAN);
        }
        else
        {
            std::sort(depths.begin(), depths.end());
            float median_depth = depths[depths.size() / 2];

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Determine roi location

            rgbd::View view(*last_image, last_image->getDepthImage().cols);
            geo::Vec3 pos = view.getRasterizer().project2Dto3D(roi_depth_center.x, roi_depth_center.y) * static_cast<double>(median_depth);
            pos.y = -pos.y;
            pos.z = -pos.z;

            ROS_INFO("Pose (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
            geo::convert(pos, point_msg.point);
        }

        res.points.push_back(point_msg);
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_3d_point_from_image_roi");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double max_fps = 30;
    nh_private.getParam("max_fps", max_fps);

    // Listener
    rgbd::Client client;

    client.intialize(ros::names::resolve("rgbd"));

    g_last_images_.set_capacity(100);

    // srv
    ros::ServiceServer srv_project_2d_to_3d = nh.advertiseService("project_2d_to_3d", srvGet3dPointFromROI);
    ros::Time last_image_stamp;

    ros::Rate r(max_fps);
    while (ros::ok())
    {
        rgbd::Image image;
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                last_image_stamp = ros::Time(image.getTimestamp());

                g_last_images_.push_back(std::make_shared<rgbd::Image>(image));
                ROS_DEBUG("New image added to buffer");
            }
        }
        ros::spinOnce(); // Process service request after getting a new image
        if (!last_image_stamp.isZero() && ros::Time::now() - last_image_stamp > ros::Duration(5.0))
        {
            ROS_ERROR("No new images received images for 5 seconds, ...restarting");
            exit(1);
        }
        r.sleep();
    }
}
