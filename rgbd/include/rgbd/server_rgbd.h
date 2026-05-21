#ifndef RGBD_SERVER_RGBD_H_
#define RGBD_SERVER_RGBD_H_

#include <rclcpp/rclcpp.hpp>

#include <rgbd_interfaces/msg/rgbd.hpp>
#include <rgbd_interfaces/srv/get_rgbd.hpp>

#include "rgbd/image.h"

#include <mutex>
#include <thread>

namespace rgbd
{

/**
 * @brief Server which provides RGBD topic and RGBD service
 */
class ServerRGBD
{
  public:
    /**
     * @brief Constructor
     */
    explicit ServerRGBD(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     *
     * Service thread is stopped and joined
     */
    virtual ~ServerRGBD();

    /**
     * @brief Initialize server
     * @param name Fully resolved server name
     * @param rgb_type rgb storage type
     * @param depth_type depth storage type
     * @param service_freq frequency of the thread processing service requests
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS,
                    DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS, float service_freq = 10.0f);

    /**
     * @brief Write a new image to all interfaces
     * @param image Image to be written
     */
    void send(const Image& image);

    /**
     * @brief version of the RGBD message being used
     */
    static const int MESSAGE_VERSION;

  protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rgbd_interfaces::msg::RGBD>::SharedPtr pub_image_;
    rclcpp::Service<rgbd_interfaces::srv::GetRGBD>::SharedPtr service_server_;

    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

    rgbd::Image image_;
    std::mutex image_mutex_;

    // Service thread
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread service_thread_;
    bool stop_service_thread_{false};

    /**
     * @brief service callback
     * @param req Service Request
     * @param resp Service Response
     */
    void serviceCallback(const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Request> req,
                         std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Response> resp);
    /**
     * @brief Function to be called in the thread providing the service
     * @param frequency frequency for checking service requests
     */
    void serviceThreadFunc(float frequency);
};

} // namespace rgbd

#endif // RGBD_SERVER_RGBD_H_
