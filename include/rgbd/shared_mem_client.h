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

    bool intialize(const std::string& server_name);

    bool initialized();

    bool nextImage(Image& image);

private:

    boost::interprocess::shared_memory_object shm;

    boost::interprocess::mapped_region mem_buffer_header;
    boost::interprocess::mapped_region mem_image;

    int sequence_nr;

    BufferHeader* buffer_header;

};

} // end namespace rgbd

#endif
