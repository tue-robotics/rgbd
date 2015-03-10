#include "rgbd/shared_mem_client.h"

#include "rgbd/Image.h"

namespace ipc = boost::interprocess;

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

SharedMemClient::SharedMemClient() : buffer_header(0)
{
}

// ----------------------------------------------------------------------------------------------------

SharedMemClient::~SharedMemClient()
{
}

// ----------------------------------------------------------------------------------------------------

bool SharedMemClient::intialize(const std::string& server_name)
{
    try
    {

        // Open already created shared memory object.
        shm = ipc::shared_memory_object(ipc::open_only, server_name.c_str(), ipc::read_write);

        mem_buffer_header = ipc::mapped_region(shm, ipc::read_write, 0, sizeof(BufferHeader));
        mem_image = ipc::mapped_region(shm, ipc::read_only, sizeof(BufferHeader));

        buffer_header = static_cast<BufferHeader*>(mem_buffer_header.get_address());

        sequence_nr = 0;
    }
    catch(ipc::interprocess_exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool SharedMemClient::initialized()
{
    return (buffer_header != 0);
}

// ----------------------------------------------------------------------------------------------------

bool SharedMemClient::nextImage(Image& image)
{
    if (buffer_header->sequence_nr == sequence_nr)
        return false;

    cv::Mat rgb, depth;

    ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header->mutex);

    //    if (buffer_header.sequence_nr == sequence_nr)
    //        buffer_header.cond_empty.wait(lock);

    uchar* image_data = static_cast<uchar*>(mem_image.get_address());

    uint64_t rgb_data_size = buffer_header->rgb_width * buffer_header->rgb_height * 3;
    uint64_t depth_data_size = buffer_header->depth_width * buffer_header->depth_height * 4;

    rgb = cv::Mat(buffer_header->rgb_height, buffer_header->rgb_width, CV_8UC3);
    depth = cv::Mat(buffer_header->depth_height, buffer_header->depth_width, CV_32FC1);

    memcpy(rgb.data, image_data, rgb_data_size);
    memcpy(depth.data, image_data + rgb_data_size, depth_data_size);

    image.depth_image_ = depth;
    image.rgb_image_ = rgb;

    sequence_nr = buffer_header->sequence_nr;
}

} // end namespace rgbd

