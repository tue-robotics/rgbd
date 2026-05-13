#ifndef RGBD_SERVER_RGBD_H_
#define RGBD_SERVER_RGBD_H_

#include <rclcpp/rclcpp.hpp>

#include <rgbd_interfaces/msg/rgbd.hpp>
#include <rgbd_interfaces/srv/get_rgbd.hpp>

#include "rgbd/image.h"

#include <mutex>
#include <thread>

namespace rgbd {

class ServerRGBD {
public:
    explicit ServerRGBD(const rclcpp::Node::SharedPtr& node = nullptr);
    virtual ~ServerRGBD();

    void initialize(const std::string& name,
                    RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS,
                    DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS,
                    float service_freq = 10.0f);

    void send(const Image& image);

    static const int MESSAGE_VERSION;

protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rgbd_interfaces::msg::RGBD>::SharedPtr pub_image_;
    rclcpp::Service<rgbd_interfaces::srv::GetRGBD>::SharedPtr service_server_;

    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

    rgbd::Image image_;
    std::mutex image_mutex_;

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread service_thread_;
    bool stop_service_thread_{false};

    void serviceCallback(const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Request> req,
                         std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Response> resp);
    void serviceThreadFunc(float frequency);
};

}  // namespace rgbd

#endif  // RGBD_SERVER_RGBD_H_
