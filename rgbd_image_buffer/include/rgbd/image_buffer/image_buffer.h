#ifndef RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_
#define RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_

#include <geolib/datatypes.h>

#include <rclcpp/rclcpp.hpp>

#include <rgbd/types.h>

#include <tf2_ros/buffer.h>

#include <forward_list>
#include <memory>
#include <mutex>
#include <thread>

namespace tf2_ros { class TransformListener; }

namespace rgbd
{
class Client;

class ImageBuffer
{
public:
    explicit ImageBuffer(const rclcpp::Node::SharedPtr& node = nullptr);
    ~ImageBuffer();

    void initialize(const std::string& topic,
                    const std::string& root_frame = "map",
                    float worker_thread_frequency = 20);

    bool nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);
    bool waitForRecentImage(rgbd::ImageConstPtr& image,
                            geo::Pose3D& sensor_pose,
                            double timeout_sec,
                            double check_rate);
    bool waitForRecentImage(rgbd::ImageConstPtr& image,
                            geo::Pose3D& sensor_pose,
                            double timeout_sec,
                            uint timeout_tries = 25u);

private:
    rclcpp::Node::SharedPtr node_;
    std::string root_frame_;

    std::unique_ptr<rgbd::Client> rgbd_client_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::forward_list<rgbd::ImageConstPtr> image_buffer_;

    std::pair<rgbd::ImageConstPtr, geo::Pose3D> recent_image_;
    std::mutex recent_image_mutex_;
    std::unique_ptr<std::thread> worker_thread_ptr_;
    bool shutdown_;

    bool getMostRecentImageTF();
    void workerThreadFunc(float frequency = 20);
};

} // namespace rgbd

#endif // RGBD_IMAGE_BUFFER_IMAGE_BUFFER_H_
