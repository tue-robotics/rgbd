/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_H_
#define RGBD_CLIENT_ROS_H_

#include <rclcpp/rclcpp.hpp>

#include "rgbd/client_ros_base.h"
#include "rgbd/types.h"

namespace rgbd {

/**
 * @brief Client which subscribes to regular ROS image topics
 */
class ClientROS : public ClientROSBase {
public:
    /**
     * @brief Constructor
     */
    explicit ClientROS(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     */
    ~ClientROS() override;

    /**
     * @brief Initialize the subscriber
     * @param rgb_image_topic topic name of the rgb image; topic will still be resolved.
     * @param depth_image_topic topic name of the depth image; topic will still be resolved.
     * @param cam_info_topic topic name of the camera info; topic will still be resolved.
     * @return indicates success
     */
    bool initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);

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
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_ROS_H_
