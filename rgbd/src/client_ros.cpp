#include "rgbd/client_ros.h"
#include "rgbd/image.h"
#include "rgbd/types.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp> // IWYU pragma: keep
#else
#include <message_filters/subscriber.h> // IWYU pragma: keep
#endif
#if __has_include(<message_filters/synchronizer.hpp>)
#include <message_filters/synchronizer.hpp> // IWYU pragma: keep
#else
#include <message_filters/synchronizer.h> // IWYU pragma: keep
#endif
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <utility>

namespace rgbd
{

namespace
{

template <typename SubscriberT, typename NodeT>
auto subscribeSensorDataImpl(
    SubscriberT& sub, const NodeT& node, const std::string& topic, const rclcpp::SubscriptionOptions& options, int)
    -> decltype(sub.subscribe(node, topic, rclcpp::SensorDataQoS(), options), void())
{
    sub.subscribe(node, topic, rclcpp::SensorDataQoS(), options);
}

template <typename SubscriberT, typename NodeT>
void subscribeSensorDataImpl(
    SubscriberT& sub, const NodeT& node, const std::string& topic, rclcpp::SubscriptionOptions options, int64_t)
{
    sub.subscribe(node, topic, rmw_qos_profile_sensor_data, std::move(options));
}

template <typename SubscriberT, typename NodeT>
void subscribeSensorData(SubscriberT& sub,
                         const NodeT& node,
                         const std::string& topic,
                         const rclcpp::SubscriptionOptions& options)
{
    subscribeSensorDataImpl(sub, node, topic, options, 0);
}

} // namespace

ClientROS::ClientROS(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_client_ros")), sync_(nullptr), sub_rgb_sync_(nullptr),
    sub_depth_sync_(nullptr), cb_group_(nullptr)

{
}

ClientROS::~ClientROS()
{
    deinitialize();
}

bool ClientROS::initialize(const std::string& rgb_image_topic,
                           const std::string& depth_image_topic,
                           const std::string& cam_info_topic)
{
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_;
    sub_cam_info_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg) { camInfoCallback(cam_info_msg); },
        sub_options);

    sub_rgb_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    sub_depth_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();

    subscribeSensorData(*sub_rgb_sync_, node_, rgb_image_topic, sub_options);
    subscribeSensorData(*sub_depth_sync_, node_, depth_image_topic, sub_options);

    sync_ = std::make_unique<message_filters::Synchronizer<RGBDApproxPolicy>>(
        RGBDApproxPolicy(10), *sub_rgb_sync_, *sub_depth_sync_);
    // message_filters::Synchronizer::registerCallback re-wraps the callback in a fixed 9-placeholder std::bind
    // internally; a fixed-arity lambda cannot satisfy that call and fails to compile.
    // NOLINTNEXTLINE(modernize-avoid-bind)
    sync_->registerCallback(std::bind(&ClientROS::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    return true;
}

bool ClientROS::deinitialize()
{
    executor_.remove_callback_group(cb_group_);
    sync_.reset();
    sub_rgb_sync_.reset();
    sub_depth_sync_.reset();
    sub_cam_info_.reset();
    cb_group_.reset();
    cam_model_ = image_geometry::PinholeCameraModel();
    return true;
}

void ClientROS::camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg)
{
    if (!cam_model_.initialized())
    {
        cam_model_.fromCameraInfo(*cam_info_msg);
        sub_cam_info_.reset();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"),
                     "CameraInfo should unsubscribe after initializing the camera model");
    }
}

bool ClientROS::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                              const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg)
{
    if (!cam_model_.initialized())
    {
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("ClientROS"), *node_->get_clock(), 1000, "ClientROS: cam_model not yet initialized");
        return false;
    }

    cv_bridge::CvImagePtr rgb_img_ptr;
    cv_bridge::CvImagePtr depth_img_ptr;

    try
    {
        rgb_img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"), "ClientROS: Could not deserialize rgb image: %s", e.what());
        return false;
    }

    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, depth_image_msg->encoding);

        if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            cv::Mat depth_image(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1);
            for (int x = 0; x < depth_image.cols; ++x)
            {
                for (int y = 0; y < depth_image.rows; ++y)
                {
                    depth_image.at<float>(y, x) = static_cast<float>(depth_img_ptr->image.at<uint16_t>(y, x)) / 1000.0f;
                }
            }
            depth_img_ptr->image = depth_image;
        }
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"), "ClientROS: Could not deserialize depth image: %s", e.what());
        return false;
    }

    if (!image_ptr_)
    {
        image_ptr_ = new Image();
    }

    image_ptr_->setRGBImage(rgb_img_ptr->image);
    image_ptr_->setDepthImage(depth_img_ptr->image);
    image_ptr_->setCameraModel(cam_model_);
    image_ptr_->setFrameId(rgb_image_msg->header.frame_id);
    image_ptr_->setTimestamp(rclcpp::Time(rgb_image_msg->header.stamp).seconds());
    new_image_ = true;

    return true;
}

bool ClientROS::nextImage(Image& image)
{
    new_image_ = false;
    image_ptr_ = &image;
    executor_.spin_some();
    return new_image_;
}

ImagePtr ClientROS::nextImage()
{
    new_image_ = false;
    image_ptr_ = nullptr;
    executor_.spin_some();
    if (!new_image_)
    {
        delete image_ptr_;
        return nullptr;
    }
    return ImagePtr(image_ptr_);
}

} // namespace rgbd
