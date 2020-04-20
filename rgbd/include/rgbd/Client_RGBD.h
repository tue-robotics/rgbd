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
     * image_ptr_ is not delete as the client never owns the image pointer
     */
    virtual ~ClientRGBD();

    /**
     * @brief intialize Initialize the client
     * @param server_name Fully resolved server name
     */
    void intialize(const std::string& server_name);

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
     * @brief new_image_ Track if image is updated in a callback.
     */
    bool new_image_;
    Image* image_ptr_;

    void rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg);

};

}

#endif // RGBD_CLIENT_RGBD_H_
