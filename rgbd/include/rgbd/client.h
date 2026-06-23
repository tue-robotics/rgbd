/**
 * This client listens to RGBD messages on a single topic or via shared memory, provides rgbd::Image.
 */

#ifndef RGBD_CLIENT_H_
#define RGBD_CLIENT_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rgbd/client_rgbd.h"
#include "rgbd/client_shm.h"
#include "rgbd/types.h"

#include <memory>
#include <mutex>
#include <thread>

namespace rgbd
{

/**
 * @brief Client which uses the interfaces of ClientRGBD and ClientSHM
 */
class Client
{
public:
    /**
     * @brief Constructor
     */
    explicit Client(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     */
    virtual ~Client();

    /**
     * @brief Initialize the client
     * @param server_name Fully resolved server name
     * @param timeout Timeout used to initialize each interface, currently only the ClientSHM interface requires a
     * timeout
     * @return indicates success
     */
    bool initialize(const std::string& server_name, float timeout = 5.0);

    /**
     * @brief Calls deinitialize on implementation clients. Shuts down both implementations. #initialized will now
     * return false.
     * @return indicates success
     */
    bool deinitialize();

    /**
     * @brief Check if the client is initialized.
     * nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() const { return !server_name_.empty(); }

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
    enum class ClientImplMode
    {
        shm,
        rgbd
    };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_shm_hosts_;
    rclcpp::CallbackGroup::SharedPtr cb_group_shm_hosts_;

    ClientRGBD client_rgbd_;
    ClientSHM client_shm_;

    std::string hostname_;
    std::string server_name_;

    // Last heartbeat time of shared memory server on this host
    rclcpp::Time last_time_shm_server_online_;

    std::thread sub_hosts_thread_;
    bool stop_sub_hosts_thread_;

    ClientImplMode client_impl_mode_;
    std::mutex switch_impl_mutex_;

    void hostsCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void subHostsThreadFunc(float frequency);
};

} // namespace rgbd

#endif // RGBD_CLIENT_H_
