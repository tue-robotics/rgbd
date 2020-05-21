/**
 * This client listens to RGBD messages on a single topic directing rgbd::Image.
 */

#ifndef RGBD_CLIENT_RGBD_H_
#define RGBD_CLIENT_RGBD_H_

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include "rgbd/types.h"

#include "rgbd_msgs/RGBD.h"


namespace rgbd {

/**
 * @brief Client which subscribes to RGBD topic
 */
class ClientRGBD {

public:

    /**
     * @brief Constructor
     */
    ClientRGBD();

    /**
     * @brief Destructor
     *
     * image_ptr_ is not deleted as the client never owns the image pointer
     */
    virtual ~ClientRGBD();

    /**
     * @brief intialize Initialize the client
     * @param server_name Fully resolved server name
     * @return indicates success
     */
    bool intialize(const std::string& server_name);

    /**
     * @brief Check if the client is initialized. nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return !sub_image_.getTopic().empty(); }

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

    ros::NodeHandle nh_;

    ros::Subscriber sub_image_;
    ros::CallbackQueue cb_queue_;

    /**
     * @brief Track if image is updated in a callback.
     */
    bool new_image_;
    /**
     * @brief Pointer to the Image being written in the NextImage calls. Either set to the address of the provided reference.
     * Or being wrapped into a shared pointer. Ownership therefore belongs to the caller of nextImage, which provides the references or receives the SharedPtr.
     * image_ptr_ should only be accessed inside a NextImage call. Outside it, the image_ptr_ might be invalid/
     * This is used since you can not pass additional arguments to the callback. A raw pointer is prefered to avoid unnecessary copy operations.
     */
    Image* image_ptr_;

    void rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg);

};

}

#endif // RGBD_CLIENT_RGBD_H_
