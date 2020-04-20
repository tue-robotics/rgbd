#ifndef RGBD_CLIENT_SHM_H_
#define RGBD_CLIENT_SHM_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd/image_header.h"
#include "rgbd/types.h"

namespace rgbd
{

/**
 * @brief Client which uses shared memory, requires a server on the same machine
 */
class ClientSHM
{

public:

    /**
     * @brief Constructor
     *
     * buffer_header pointer is initialized to nullptr
     */
    ClientSHM();

    /**
     * @brief Destructor
     *
     * buffer_header is not deleted as the client doesn't close the shared memory
     */
    ~ClientSHM();

    /**
     * @brief intialize Initialize shared memory client
     * @param server_name Fully resolved server name
     * @param timeout Timeout to wait for shared memory server
     * @return indicates success
     */
    bool intialize(const std::string& server_name, float timeout = 5.0);

    /**
     * @brief Check if the client is initialized. nextImage shouldn't be called if client is not initialized.
     * @return initialized or not
     */
    bool initialized();

    /**
     * @brief Get a new Image. If no new image has been received, the sequence nummer is still the same as the previous call,
     * no image will be written and false will be returned.
     * @param image Image reference which will be written.
     * @return valid image written
     */
    bool nextImage(Image& image);

private:

    boost::interprocess::shared_memory_object shm;

    boost::interprocess::mapped_region mem_buffer_header;
    boost::interprocess::mapped_region mem_image;

    /**
     * @brief sequence_nr Contains the sequence nummer of the last NextImage call
     */
    uint64_t sequence_nr;

    BufferHeader* buffer_header;

};

} // end namespace rgbd

#endif
