#include "rgbd/image_buffer/image_buffer.h"
#include <algorithm>

#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rgbd/client.h>
#include <rgbd/image.h>

#if __has_include(<tf2/time.hpp>)
#include <tf2/time.hpp>
#else
#include <tf2/time.h>
#endif
#include <tf2_ros/transform_listener.h>

namespace rgbd
{

ImageBuffer::ImageBuffer(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_image_buffer")), rgbd_client_(nullptr),
    tf_buffer_(node_->get_clock()), tf_listener_(nullptr), shutdown_(false)
{
}

ImageBuffer::~ImageBuffer()
{
    shutdown_ = true;
    if (worker_thread_ptr_)
        worker_thread_ptr_->join();
}

void ImageBuffer::initialize(const std::string& topic, const std::string& root_frame, float worker_thread_frequency)
{
    root_frame_ = root_frame;

    if (!rgbd_client_)
        rgbd_client_ = std::make_unique<rgbd::Client>(node_);

    rgbd_client_->initialize(topic);

    if (!tf_listener_)
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    worker_thread_ptr_ = std::make_unique<std::thread>(&ImageBuffer::workerThreadFunc, this, worker_thread_frequency);
}

bool ImageBuffer::waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec,
                                     double check_rate)
{
    if (!rgbd_client_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_buffer"), "[IMAGE_BUFFER] No RGBD client");
        return false;
    }

    const rclcpp::Time t_start = node_->now();
    const rclcpp::Time t_end = t_start + rclcpp::Duration::from_seconds(timeout_sec);
    if (check_rate <= 0)
    {
        check_rate = 10.;
    }
    rclcpp::Rate r(check_rate);

    rgbd::ImageConstPtr rgbd_image;
    do
    {
        rgbd_image = rgbd_client_->nextImage();

        if (rgbd_image)
        {
            break;
        }
        else if (node_->now() > t_end)
        {
            RCLCPP_ERROR(rclcpp::get_logger("image_buffer"), "[IMAGE_BUFFER] timeout waiting for rgbd image");
            return false;
        }
        else
            r.sleep();
    } while (rclcpp::ok());

    const rclcpp::Time image_stamp = rclcpp::Time(static_cast<int64_t>(rgbd_image->getTimestamp() * 1e9));
    if (!tf_buffer_.canTransform(root_frame_, rgbd_image->getFrameId(), image_stamp))
    {
        if (!tf_buffer_.canTransform(root_frame_, rgbd_image->getFrameId(), image_stamp,
                                     tf2::durationFromSec(std::max(0.0, (t_end - node_->now()).seconds()))))
        {
            RCLCPP_ERROR(rclcpp::get_logger("image_buffer"), "[IMAGE_BUFFER] timeout waiting for tf");
            return false;
        }
    }

    try
    {
        geometry_msgs::msg::TransformStamped t_sensor_pose =
            tf_buffer_.lookupTransform(root_frame_, rgbd_image->getFrameId(), image_stamp);
        geo::convert(t_sensor_pose.transform, sensor_pose);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_buffer"), "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        return false;
    }

    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    image = rgbd_image;

    return true;
}

bool ImageBuffer::waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec,
                                     uint timeout_tries)
{
    if (timeout_tries <= 0)
    {
        timeout_tries = 25;
    }
    double freq = timeout_sec > 0 ? timeout_tries / timeout_sec : 1000;

    return waitForRecentImage(image, sensor_pose, timeout_sec, freq);
}

bool ImageBuffer::nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose)
{
    std::lock_guard<std::mutex> lg(recent_image_mutex_);
    if (!recent_image_.first)
    {
        return false;
    }

    image = recent_image_.first;
    sensor_pose = recent_image_.second;

    recent_image_.first.reset();

    return true;
}

bool ImageBuffer::getMostRecentImageTF()
{
    if (!rgbd_client_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_buffer"), "[IMAGE_BUFFER] No RGBD client");
        return false;
    }

    {
        rgbd::ImageConstPtr new_image = rgbd_client_->nextImage();
        if (new_image)
        {
            image_buffer_.push_front(new_image);
        }
    }

    geo::Pose3D sensor_pose;

    for (auto it = image_buffer_.begin(); it != image_buffer_.end(); ++it)
    {
        rgbd::ImageConstPtr& rgbd_image = *it;
        try
        {
            geometry_msgs::msg::TransformStamped t_sensor_pose =
                tf_buffer_.lookupTransform(root_frame_,
                                           rgbd_image->getFrameId(),
                                           rclcpp::Time(static_cast<int64_t>(rgbd_image->getTimestamp() * 1e9)));
            geo::convert(t_sensor_pose.transform, sensor_pose);
        }
        catch (tf2::ExtrapolationException& ex)
        {
            try
            {
                geometry_msgs::msg::TransformStamped latest_sensor_pose =
                    tf_buffer_.lookupTransform(root_frame_, rgbd_image->getFrameId(), tf2::TimePointZero);
                if (rclcpp::Time(latest_sensor_pose.header.stamp) >
                    rclcpp::Time(static_cast<int64_t>(rgbd_image->getTimestamp() * 1e9)))
                {
                    image_buffer_.erase_after(it, image_buffer_.end());
                    return false;
                }
                else
                {
                    (void)ex;
                    continue;
                }
            }
            catch (tf2::TransformException&)
            {
                continue;
            }
        }
        catch (tf2::TransformException&)
        {
            continue;
        }

        sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

        {
            std::lock_guard<std::mutex> lg(recent_image_mutex_);
            recent_image_.first = rgbd_image;
            recent_image_.second = sensor_pose;
        }

        image_buffer_.erase_after(it, image_buffer_.end());

        return true;
    }

    return false;
}

void ImageBuffer::workerThreadFunc(float frequency)
{
    rclcpp::Rate r(frequency);
    while (!shutdown_)
    {
        getMostRecentImageTF();
        r.sleep();
    }
}

} // namespace rgbd
