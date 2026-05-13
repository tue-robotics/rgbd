#ifndef RGBD_SERVER_SHM_H_
#define RGBD_SERVER_SHM_H_

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

#include <rclcpp/rclcpp.hpp>

#include "rgbd/image_header.h"
#include "rgbd/types.h"

#include <memory>
#include <thread>

namespace rgbd
{

class ServerSHM
{
public:
    explicit ServerSHM(const rclcpp::Node::SharedPtr& node = nullptr);
    ~ServerSHM();

    void initialize(const std::string& name);
    void send(const Image& image);

private:
    std::string shared_mem_name_;

    boost::interprocess::shared_memory_object shm_;

    boost::interprocess::mapped_region mem_buffer_header_;
    boost::interprocess::mapped_region mem_image_;

    BufferHeader* buffer_header_;
    unsigned char* image_data_;

    uint64_t rgb_data_size_;
    uint64_t depth_data_size_;
    uint64_t image_data_size_;

    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<std::thread> check_shm_thread_ptr_;
    bool stop_check_shm_thread_{false};

    void checkSHMThreadFunc(float frequency);
};

void pubHostnameThreadFunc(const rclcpp::Node::SharedPtr& node, const std::string& server_name, const std::string& hostname, float frequency);

} // namespace rgbd

#endif // RGBD_SERVER_SHM_H_
