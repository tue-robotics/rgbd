#ifndef RGBD_SHARED_MEM_CLIENT_H_
#define RGBD_SHARED_MEM_CLIENT_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd/image_header.h"
#include "rgbd/types.h"

namespace rgbd
{

class SharedMemClient
{

public:

    SharedMemClient();

    ~SharedMemClient();

    /**
     * @brief intialize Initialize shared memory client
     * @param server_name Fully resolved server name
     * @param timeout Timeout to wait for shared memory server
     * @return indicates success
     */
    bool intialize(const std::string& server_name, float timeout = 5.0);

    bool initialized();

    bool nextImage(Image& image);

private:

    boost::interprocess::shared_memory_object shm;

    boost::interprocess::mapped_region mem_buffer_header;
    boost::interprocess::mapped_region mem_image;

    uint64_t sequence_nr;

    BufferHeader* buffer_header;

};

} // end namespace rgbd

#endif
