#ifndef RGBD_SERVER_SHM_H_
#define RGBD_SERVER_SHM_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <ros/node_handle.h>

#include "rgbd/image_header.h"
#include "rgbd/types.h"


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
    ServerSHM();

    /**
     * @brief Destructor
     *
     * Shared memory object is deleted, buffer_header_ and image_data_ are also deleted
     */
    ~ServerSHM();

    /**
     * @brief initialize Initialize shared memory server
     * @param name Fully resolved server name
     */
    void initialize(const std::string& name);

    /**
     * @brief send Write a new image to the shared memory
     * @param image Image to be written to the shared memory
     */
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

};

void pubHostnameThreadFunc(ros::NodeHandle& nh, const std::string server_name, const std::string hostname, const float frequency);

} // end namespace rgbd

#endif // RGBD_SERVER_SHM_H_
