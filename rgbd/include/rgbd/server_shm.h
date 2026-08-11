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

/**
 * @brief Server which uses shared memory, this only works for clients on the same machine
 */
class ServerSHM
{
public:
    /**
     * @brief Constructor
     *
     * buffer_header_ and image_data_ pointers are initialized to nullptr
     */
    explicit ServerSHM(const rclcpp::Node::SharedPtr& node = nullptr);

    /**
     * @brief Destructor
     *
     * Shared memory object is deleted
     */
    ~ServerSHM();

    /**
     * @brief initialize shared memory server
     * @param name Fully resolved server name
     */
    void initialize(const std::string& name);

    /**
     * @brief Write a new image to the shared memory
     * @param image Image to be written to the shared memory
     */
    void send(const Image& image);

private:
    std::string shared_mem_name_;

    boost::interprocess::shared_memory_object shm_;

    boost::interprocess::mapped_region mem_buffer_header_;
    boost::interprocess::mapped_region mem_image_;

    BufferHeader* buffer_header_{nullptr};
    unsigned char* image_data_{nullptr};

    uint64_t rgb_data_size_{};
    uint64_t depth_data_size_{};
    uint64_t image_data_size_{};

    rclcpp::Node::SharedPtr node_;

    // SHM check thread
    std::unique_ptr<std::thread> check_shm_thread_ptr_;
    bool stop_check_shm_thread_{false};

    /**
     * @brief Check if the SHM can be opened
     * @param frequency Frequency of checking
     */
    void checkSHMThreadFunc(float frequency);
};

/**
 * @brief Publish the host that serves SHM for a server name.
 */
void pubHostnameThreadFunc(const rclcpp::Node::SharedPtr& node,
                           const std::string& server_name,
                           const std::string& hostname,
                           float frequency);

} // namespace rgbd

#endif // RGBD_SERVER_SHM_H_
