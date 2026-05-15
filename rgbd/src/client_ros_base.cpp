#include "rgbd/client_ros_base.h"
#include "rgbd/image.h"

#include <cv_bridge/cv_bridge.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

namespace rgbd {

ClientROSBase::ClientROSBase(const rclcpp::Node::SharedPtr& node)
    : node_(node ? node : rclcpp::Node::make_shared("rgbd_client_ros_base"))
    , sync_(nullptr)
    , sub_rgb_sync_(nullptr)
    , sub_depth_sync_(nullptr)
    , new_image_(false)
    , image_ptr_(nullptr)
{
}

ClientROSBase::~ClientROSBase()
{
    deinitialize();
}

bool ClientROSBase::initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    sub_cam_info_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ClientROSBase::camInfoCallback, this, std::placeholders::_1));

    sub_rgb_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    sub_depth_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();

    sub_rgb_sync_->subscribe(node_.get(), rgb_image_topic, rmw_qos_profile_sensor_data);
    sub_depth_sync_->subscribe(node_.get(), depth_image_topic, rmw_qos_profile_sensor_data);

    sync_ = std::make_unique<message_filters::Synchronizer<RGBDApproxPolicy>>(RGBDApproxPolicy(10), *sub_rgb_sync_, *sub_depth_sync_);

    return true;
}

bool ClientROSBase::deinitialize()
{
    sync_.reset();
    sub_rgb_sync_.reset();
    sub_depth_sync_.reset();
    sub_cam_info_.reset();
    cam_model_ = image_geometry::PinholeCameraModel();
    return true;
}

void ClientROSBase::camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg)
{
    if (!cam_model_.initialized())
    {
        cam_model_.fromCameraInfo(*cam_info_msg);
        sub_cam_info_.reset();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"), "CameraInfo should unsubscribe after initializing the camera model");
    }
}

bool ClientROSBase::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg)
{
    if (!cam_model_.initialized())
    {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ClientROS"), *node_->get_clock(), 1000, "ClientROSBase: cam_model not yet initialized");
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
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"), "ClientROSBase: Could not deserialize rgb image: %s", e.what());
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
                    depth_image.at<float>(y, x) = static_cast<float>(depth_img_ptr->image.at<unsigned short>(y, x)) / 1000.0f;
                }
            }
            depth_img_ptr->image = depth_image;
        }
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ClientROS"), "ClientROSBase: Could not deserialize depth image: %s", e.what());
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

}  // namespace rgbd
