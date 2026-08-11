/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_H_
#define RGBD_CLIENT_ROS_H_

#include <rclcpp/rclcpp.hpp>

#if __has_include(<message_filters/sync_policies/approximate_time.hpp>)
#include <message_filters/sync_policies/approximate_time.hpp>
#else
#include <message_filters/sync_policies/approximate_time.h>
#endif
#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#else
#include <message_filters/subscriber.h>
#endif
#if __has_include(<message_filters/synchronizer.hpp>)
#include <message_filters/synchronizer.hpp>
#else
#include <message_filters/synchronizer.h>
#endif
#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rgbd/types.h"

#include <memory>

namespace rgbd
{

using RGBDApproxPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

/**
 * @brief Client which subscribes to regular ROS image topics
 */
class ClientROS
{
public:
    /**
     * @brief Constructor
     */
    explicit ClientROS(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     */
    ~ClientROS();

    /**
     * @brief Initialize the subscriber
     * @param rgb_image_topic topic name of the rgb image; topic will still be resolved.
     * @param depth_image_topic topic name of the depth image; topic will still be resolved.
     * @param cam_info_topic topic name of the camera info; topic will still be resolved.
     * @return indicates success
     */
    bool initialize(const std::string& rgb_image_topic,
                    const std::string& depth_image_topic,
                    const std::string& cam_info_topic);

    /**
     * @brief Clears the subscribers. #initialized will now return false.
     * @return indicates success
     */
    bool deinitialize();

    /**
     * @brief Check if the client is initialized. nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() const { return static_cast<bool>(sync_); }

    /**
     * @brief Get a new Image. If no new image has been received since the last call,
     * no image will be written and false will be returned.
     * @param image Image reference which will be written.
     * @return valid image written
     */
    bool nextImage(Image& image);

    /**
     * @brief Get a new Image. If no new image has been received since the last call,
     * The ImagePtr will be a nullptr
     * @return ImagePtr to an Image or a nullptr
     */
    ImagePtr nextImage();

private:
    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<message_filters::Synchronizer<RGBDApproxPolicy>> sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_rgb_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_sync_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    image_geometry::PinholeCameraModel cam_model_;

    /**
     * @brief Track if image is updated in a callback.
     */
    bool new_image_{false};
    /**
     * @brief Pointer to image. Image could be provided by reference or wrapped in a shared_ptr.
     * This class never takes ownership.
     */
    Image* image_ptr_{nullptr};

    /**
     * @brief Callback for CameraInfo, will unsubscribe after successfully receiving first message.
     * @param cam_info_msg message
     */
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg);

    /**
     * @brief Callback for synchronized rgb and depth image
     * @param rgb_image_msg rgb image message
     * @param depth_image_msg depth image message
     * @return success
     */
    bool imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg);
};

} // namespace rgbd

#endif // RGBD_CLIENT_ROS_H_
