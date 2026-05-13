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

namespace rgbd {

class Client {
public:
    explicit Client(const rclcpp::Node::SharedPtr& node = nullptr);
    virtual ~Client();

    bool initialize(const std::string& server_name, float timeout = 5.0);
    bool deinitialize();

    bool initialized() const { return !server_name_.empty(); }

    bool nextImage(Image& image);
    ImagePtr nextImage();

protected:
    enum class ClientImplMode { shm, rgbd };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_shm_hosts_;

    ClientRGBD client_rgbd_;
    ClientSHM client_shm_;

    std::string hostname_;
    std::string server_name_;

    rclcpp::Time last_time_shm_server_online_;

    std::thread sub_hosts_thread_;
    bool stop_sub_hosts_thread_;

    ClientImplMode client_impl_mode_;
    std::mutex switch_impl_mutex_;

    void hostsCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void subHostsThreadFunc(float frequency);
};

}  // namespace rgbd

#endif  // RGBD_CLIENT_H_
