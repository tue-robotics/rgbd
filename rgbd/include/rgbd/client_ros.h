/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_H_
#define RGBD_CLIENT_ROS_H_

#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "rgbd/types.h"


namespace rgbd {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDApproxPolicy;

/**
 * @brief Client which subscribes to regular ROS image topics
 */
class ClientROS {

public:

    /**
     * @brief Constructor
     *
     * Pointers are initialized to nullptr
     */
    ClientROS();

    /**
     * @brief Destructor
     *
     * image_ptr_ is not delete as the client never owns the image pointer
     */
    virtual ~ClientROS();

    /**
     * @brief Initialize the subscriber
     * @param rgb_image_topic topic name of the rgb image; topic will still be resolved.
     * @param depth_image_topic topic name of the depth image; topic will still be resolved.
     * @param cam_info_topic topic name of the camera info; topic will still be resolved.
     * @return indicates success
     */
    bool intialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);

    /**
     * @brief Check if the client is initialized. nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return (sync_); }

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
    ros::CallbackQueue cb_queue_;

    message_filters::Synchronizer<RGBDApproxPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_rgb_sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_depth_sync_;
    ros::Subscriber sub_cam_info_;
    image_geometry::PinholeCameraModel cam_model_;

    /**
     * @brief new_image_ Track if image is updated in a callback.
     */
    bool new_image_;
    /**
     * @brief image_ptr_ Pointer to image. Image could be provided by reference or the pointer will be wrapped in a shared_ptr.
     * Either this class will never take ownership.
     */
    Image* image_ptr_;

    /**
     * @brief Callback for CameraInfo, will unsubscribe after succesfully receiving first message.
     * @param cam_info_msg message
     */
    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    /**
     * @brief Callback for synched rgb and depth image
     * @param rgb_image_msg rgb image message
     * @param depth_image_msg depth image message
     */
    void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg);

};

}

#endif // RGBD_CLIENT_ROS_H_
