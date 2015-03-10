#ifndef RGBD_SHARED_MEM_SERVER_H_
#define RGBD_SHARED_MEM_SERVER_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd/image_header.h"
#include "rgbd/types.h"

namespace rgbd
{

class SharedMemServer
{

public:

    SharedMemServer();

    ~SharedMemServer();

    void initialize(const std::string& name);

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
