#include "rgbd/shared_mem_server.h"

#include "rgbd/Image.h"
#include "rgbd/View.h"

#include <boost/interprocess/sync/scoped_lock.hpp>

namespace ipc = boost::interprocess;

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

SharedMemServer::SharedMemServer() : buffer_header(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

SharedMemServer::~SharedMemServer()
{
    if (!shared_mem_name_.empty())
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());
}

// ----------------------------------------------------------------------------------------------------

void SharedMemServer::initialize(const std::string& name)
{
    shared_mem_name_ = name;
    std::replace(shared_mem_name_.begin(), shared_mem_name_.end(), '/', '-');
}

// ----------------------------------------------------------------------------------------------------

void SharedMemServer::send(const Image& image)
{
    if (shared_mem_name_.empty())
    {
        std::cout << "rgbd::SharedMemServer is not initialized" << std::endl;
        return;
    }

    const cv::Mat& rgb = image.getRGBImage();
    const cv::Mat& depth = image.getDepthImage();

    uint64_t rgb_data_size = rgb.cols * rgb.rows * 3;
    uint64_t depth_data_size = depth.cols * depth.rows * 4;
    uint64_t image_data_size = rgb_data_size + depth_data_size;

    if (!buffer_header)
    {
        // First time
        // Make sure possibly existing memory with same name is removed
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());

        //Create a shared memory object.
        shm = ipc::shared_memory_object(ipc::create_only, shared_mem_name_.c_str(), ipc::read_write);

        //Set size
        shm.truncate(sizeof(BufferHeader) + image_data_size);

        // Map buffer region
        mem_buffer_header = ipc::mapped_region(shm, ipc::read_write, 0, sizeof(BufferHeader));
        mem_image = ipc::mapped_region(shm, ipc::read_write, sizeof(BufferHeader));

        buffer_header = new (mem_buffer_header.get_address()) BufferHeader;
        buffer_header->sequence_nr = 0;

        image_data = new (mem_image.get_address()) uchar[image_data_size];

        buffer_header->rgb_width = rgb.cols;
        buffer_header->rgb_height = rgb.rows;
        buffer_header->depth_width = depth.cols;
        buffer_header->depth_height = depth.rows;

        memcpy(buffer_header->frame_id, image.getFrameId().c_str(), image.getFrameId().size() + 1);

        // CameraInfo
        const sensor_msgs::CameraInfo& cam_info = image.getCameraModel().cameraInfo();
        buffer_header->height = cam_info.height;
        buffer_header->width = cam_info.width;
        buffer_header->binning_x = cam_info.binning_x;
        buffer_header->binning_y = cam_info.binning_y;
        memcpy(buffer_header->distortion_model, cam_info.distortion_model.c_str(), cam_info.distortion_model.size() + 1);
        memcpy(buffer_header->D, &(cam_info.D), 5*sizeof(double));
        memcpy(buffer_header->K, &(cam_info.K), 9*sizeof(double));
        memcpy(buffer_header->R, &(cam_info.R), 9*sizeof(double));
        memcpy(buffer_header->P, &(cam_info.P), 12*sizeof(double));
        // CameraInfo/roi
        buffer_header->roi_x_offset = cam_info.roi.x_offset;
        buffer_header->roi_y_offset = cam_info.roi.y_offset;
        buffer_header->roi_height = cam_info.roi.height;
        buffer_header->roi_width = cam_info.roi.width;
        buffer_header->roi_do_rectify = cam_info.roi.do_rectify;
    }

    {
        ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header->mutex);

        buffer_header->timestamp = image.getTimestamp();

        memcpy(image_data, rgb.data, rgb_data_size);
        memcpy(image_data + rgb_data_size, depth.data, depth_data_size);

        buffer_header->cond_empty.notify_one();
        ++buffer_header->sequence_nr;
    }
}


} // end namespace rgbd

