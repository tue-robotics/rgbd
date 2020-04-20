#ifndef RGBD_SERVER_SHM_H_
#define RGBD_SERVER_SHM_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

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
     * buffer_header pointer is initialized to nullptr
     */
    ServerSHM();

    /**
     * @brief Destructor
     *
     * Shared memory object is deleted
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

    boost::interprocess::shared_memory_object shm;

    boost::interprocess::mapped_region mem_buffer_header;
    boost::interprocess::mapped_region mem_image;

    BufferHeader* buffer_header;

    unsigned char* image_data;

};

} // end namespace rgbd

#endif
