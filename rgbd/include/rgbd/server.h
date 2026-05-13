#ifndef RGBD_SERVER_H_
#define RGBD_SERVER_H_

#include <rclcpp/rclcpp.hpp>

#include "rgbd/image.h"
#include "rgbd/server_rgbd.h"
#include "rgbd/server_shm.h"

#include <memory>
#include <thread>

namespace rgbd {

/**
 * @brief Server which provides interfaces of ServerRGBD and ServerSHM
 */
class Server {
public:
    /**
     * @brief Constructor
     */
    explicit Server(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     */
    virtual ~Server();

    /**
     * @brief initialize server
     * @param name Fully resolved server name
     * @param rgb_type rgb storage type
     * @param depth_type depth storage type
     * @param service_freq frequency of the thread processing service requests in ServerRGBD
     */
    void initialize(const std::string& name,
                    RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS,
                    DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS,
                    float service_freq = 10.0f);

    /**
     * @brief Write a new image to all interfaces
     * @param image Image to be written
     * @param threaded Use any threaded version of send of the interfaces
     */
    void send(const Image& image, bool threaded = false);

protected:
    ServerRGBD server_rgbd_;
    ServerSHM server_shm_;

    rclcpp::Node::SharedPtr node_;

    std::string name_;
    std::string hostname_;

    // Publisher thread
    std::unique_ptr<std::thread> pub_hostname_thread_ptr_;
};

}  // namespace rgbd

#endif  // RGBD_SERVER_H_
