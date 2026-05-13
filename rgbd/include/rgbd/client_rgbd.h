/**
 * This client listens to RGBD messages on a single topic directing rgbd::Image.
 */

#ifndef RGBD_CLIENT_RGBD_H_
#define RGBD_CLIENT_RGBD_H_

#include <rclcpp/rclcpp.hpp>

#include <rgbd_interfaces/msg/rgbd.hpp>

#include "rgbd/types.h"

namespace rgbd {

/**
 * @brief Client which subscribes to RGBD topic
 */
class ClientRGBD {
public:
    /**
     * @brief Constructor
     */
    explicit ClientRGBD(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     *
     * image_ptr_ is not deleted as the client never owns the image pointer
     */
    virtual ~ClientRGBD();

    /**
     * @brief Initialize the client
     * @param server_name Fully resolved server name
     * @return indicates success
     */
    bool initialize(const std::string& server_name);

    /**
     * @brief Clears the subscriber. #initialized will now return false.
     * @return indicates success
     */
    bool deinitialize();

    /**
     * @brief Check if the client is initialized. nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() const { return static_cast<bool>(sub_image_); }

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

protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rgbd_interfaces::msg::RGBD>::SharedPtr sub_image_;

    /**
     * @brief Track if image is updated in a callback.
     */
    bool new_image_;
    /**
     * @brief Pointer to the Image being written in the NextImage calls.
     * Ownership belongs to the caller of nextImage.
     */
    Image* image_ptr_;

    void rgbdImageCallback(const rgbd_interfaces::msg::RGBD::ConstSharedPtr& msg);
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_RGBD_H_
