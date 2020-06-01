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
     * buffer_header_ and image_data_ pointers are initialized to nullptr
     */
    ClientSHM();

    /**
     * @brief Destructor
     *
     * buffer_header_ and image_data_ are not deleted as the client doesn't close the shared memory
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
     * @brief Clears all the shared memory classes to nullptrs or empty classes. #initialized will now return false.
     * @return indicates success
     */
    bool deintialize();

    /**
     * @brief Check if the client is initialized. nextImage shouldn't be called if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return (buffer_header_ != nullptr); }

    /**
     * @brief Get a new Image. If no new image has been received, the sequence nummer is still the same as the previous call,
     * no image will be written and false will be returned.
     * @param image Image reference which will be written.
     * @return valid image written
     */
    bool nextImage(Image& image);

private:

    boost::interprocess::shared_memory_object shm_;

    boost::interprocess::mapped_region mem_buffer_header_;
    boost::interprocess::mapped_region mem_image_;

    BufferHeader* buffer_header_;
    unsigned char* image_data_;

    uint64_t rgb_data_size_;
    uint64_t depth_data_size_;

    /**
     * @brief sequence_nr Contains the sequence nummer of the last NextImage call
     */
    uint64_t sequence_nr_;

};

} // end namespace rgbd

#endif // RGBD_CLIENT_SHM_H_
