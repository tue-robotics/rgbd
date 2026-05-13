#ifndef RGBD_SERVER_H_
#define RGBD_SERVER_H_

#include <rclcpp/rclcpp.hpp>

#include "rgbd/image.h"
#include "rgbd/server_rgbd.h"
#include "rgbd/server_shm.h"

#include <memory>
#include <thread>

namespace rgbd {

class Server {
public:
    explicit Server(const rclcpp::Node::SharedPtr& node = nullptr);
    virtual ~Server();

    void initialize(const std::string& name,
                    RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS,
                    DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS,
                    float service_freq = 10.0f);

    void send(const Image& image, bool threaded = false);

protected:
    ServerRGBD server_rgbd_;
    ServerSHM server_shm_;

    rclcpp::Node::SharedPtr node_;

    std::string name_;
    std::string hostname_;

    std::unique_ptr<std::thread> pub_hostname_thread_ptr_;
};

}  // namespace rgbd

#endif  // RGBD_SERVER_H_
