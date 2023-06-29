#ifndef RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_
#define RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_

#include <ros/callback_queue.h>

#include <geolib/datatypes.h>

#include <rgbd/types.h>

#include <tf2_ros/buffer.h>

#include <forward_list>
#include <memory>
#include <mutex>
#include <thread>

namespace tf2_ros
{
class TransformListener;
}

namespace rgbd
{
class Client;

/**
 * @brief The ImageBuffer class provides a buffer to synchronise rgbd images with sensor positions.
 * Images are stored until they are removed from the buffer or until a
 * certain amount of time has passed
 */
class ImageBuffer
{

public:

    ImageBuffer();

    ~ImageBuffer();

    /**
     * @brief Configure the buffer
     * @param topic ros topic on which the rgbd images are published
     * @param root_frame root frame to calculate sensor pose relative to (default: map)
     * @param worker_thread_frequency frequency at which the worker thread updates the most recent image (default: 20)
     */
    void initialize(const std::string& topic, const std::string& root_frame="map", const float worker_thread_frequency=20);

    /**
     * @brief Returns the most recent combination of image and transform, provided it is different from the previous time the function was called.
     * @param[out] image rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @return whether or not a novel image is available
     */
    bool nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

    /**
     * @brief Blocks until a new image with transform is found. Returns false if no image or TF could be found within 'timeout_sec' seconds.
     * But will always give it one try, both to get the image and to get TF.
     * @param[out] image rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @param[in] timeout_sec maximum duration to block.
     * @param[in,opt] check_rate Check for a new image on this frequency. In case =0, set to 10 Hz
     * @return whether or not a next image was available within the timeout duration.
     */
    bool waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec, double check_rate = 10.);

    /**
     * @brief Blocks until a new image with transform is found. Returns false if no image or TF could be found within 'timeout_sec' seconds.
     * But will always give it one try, both to get the image and to get TF.
     * @param[out] image rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @param[in] timeout_sec maximum duration to block.
     * @param[in] timeout_tries number of tries to check for a new image. In case =0, set to 10 tries
     * @return whether or not a next image was available within the timeout duration.
     */
    bool waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec, uint timeout_tries);

private:

    std::string root_frame_;

    std::unique_ptr<rgbd::Client> rgbd_client_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief Newer images should be pushed on the front. This will result in the front being the most recent and the back being the oldest
     */
    std::forward_list<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;

    std::pair<rgbd::ImageConstPtr, geo::Pose3D> recent_image_;
    /**
     * @brief For protecting ImageBuffer::recent_image_
     */
    std::mutex recent_image_mutex_;
    std::unique_ptr<std::thread> worker_thread_ptr_;
    bool shutdown_; // Trigger to kill the worker thread

    /**
     * @brief Iterates over the buffer and tries to get TF for the most recent image. Deletes image and all older images when succesful
     * or when image is too old from the buffer
     * @return Indicates whether the most recent image has been updated
     */
    bool getMostRecentImageTF();

    /**
     * @brief Calls ImageBuffer::getMostRecentImageTF on the specified frequency
     * @param frequency Frequency for checking new images.
     */
    void workerThreadFunc(const float frequency=20);

};

} // namespace rgbd

#endif // RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_
