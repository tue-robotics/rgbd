/**
 * This client converts rgb/depth/camera_info into RGBD::Image
 */

#ifndef RGBD_CLIENT_ROS_BASE_H_
#define RGBD_CLIENT_ROS_BASE_H_

#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include "ros/callback_queue_interface.h"

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "rgbd/types.h"

#include <memory>


namespace rgbd {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDApproxPolicy;

/**
 * @brief Client which subscribes to regular ROS image topics
 */
class ClientROSBase {

public:

    /**
     * @brief Constructor
     *
     * Pointers are initialized to nullptr
     */
    ClientROSBase(ros::NodeHandle nh);

    /**
     * @brief Destructor
     *
     * image_ptr_ is not delete as the client never owns the image pointer
     */
    virtual ~ClientROSBase();

    /**
     * @brief Initialize the subscriber
     * @param rgb_image_topic topic name of the rgb image; topic will still be resolved.
     * @param depth_image_topic topic name of the depth image; topic will still be resolved.
     * @param cam_info_topic topic name of the camera info; topic will still be resolved.
     * @return indicates success
     */
    bool initialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic);

    /**
     * @brief Clears the subscribers. #initialized will now return false.
     * @return indicates success
     */
    bool deinitialize();

    /**
     * @brief Check if the client is initialized. nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return bool(sync_); }

protected:

    ros::NodeHandle nh_;

    std::unique_ptr<message_filters::Synchronizer<RGBDApproxPolicy> > sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_rgb_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_depth_sync_;
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
     * @return success
     */
    bool imageCallback(const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::ImageConstPtr& depth_image_msg);

};

}

#endif // RGBD_CLIENT_ROS_BASE_H_
