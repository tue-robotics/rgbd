#include "rgbd/server_shm.h"

#include "rgbd/image.h"
#include "rgbd/view.h"

#include <boost/interprocess/sync/scoped_lock.hpp>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

#include <std_msgs/String.h>

namespace ipc = boost::interprocess;

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

ServerSHM::ServerSHM() : buffer_header_(nullptr), image_data_(nullptr), check_shm_thread_ptr_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

ServerSHM::~ServerSHM()
{
    nh_.shutdown();
    if (check_shm_thread_ptr_ && check_shm_thread_ptr_->joinable())
        check_shm_thread_ptr_->join();

    if (!shared_mem_name_.empty())
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());
}

// ----------------------------------------------------------------------------------------------------

void ServerSHM::initialize(const std::string& name)
{
    shared_mem_name_ = name;
    std::replace(shared_mem_name_.begin(), shared_mem_name_.end(), '/', '-');
}

// ----------------------------------------------------------------------------------------------------

void ServerSHM::send(const Image& image)
{
    if (shared_mem_name_.empty())
    {
        ROS_ERROR_NAMED("ServerSHM", "rgbd::SharedMemServer is not initialized");
        return;
    }

    const cv::Mat& rgb = image.getRGBImage();
    const cv::Mat& depth = image.getDepthImage();

    if (!buffer_header_)
    {
        // First time
        // Make sure possibly existing memory with same name is removed
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());

        //Create a shared memory object.
        shm_ = ipc::shared_memory_object(ipc::create_only, shared_mem_name_.c_str(), ipc::read_write);

        // Create SHM check thread
        if (!check_shm_thread_ptr_)
            check_shm_thread_ptr_ = std::make_unique<std::thread>(&ServerSHM::checkSHMThreadFunc, this, 1);

        // Store size
        rgb_data_size_ = static_cast<uint64_t>(rgb.cols * rgb.rows * 3);
        depth_data_size_ = static_cast<uint64_t>(depth.cols * depth.rows * 4);
        image_data_size_ = rgb_data_size_ + depth_data_size_;

        //Set size
        shm_.truncate(static_cast<ipc::offset_t>(sizeof(BufferHeader) + image_data_size_));

        // Map buffer region
        mem_buffer_header_ = ipc::mapped_region(shm_, ipc::read_write, 0, sizeof(BufferHeader));
        mem_image_ = ipc::mapped_region(shm_, ipc::read_write, sizeof(BufferHeader));

        buffer_header_ = new (mem_buffer_header_.get_address()) BufferHeader;
        buffer_header_->sequence_nr = 0;

        image_data_ = new (mem_image_.get_address()) uchar[image_data_size_];

        buffer_header_->rgb_width = rgb.cols;
        buffer_header_->rgb_height = rgb.rows;
        buffer_header_->depth_width = depth.cols;
        buffer_header_->depth_height = depth.rows;

        memcpy(buffer_header_->frame_id, image.getFrameId().c_str(), image.getFrameId().size() + 1);

        // CameraInfo
        const sensor_msgs::CameraInfo& cam_info = image.getCameraModel().cameraInfo();
        buffer_header_->height = cam_info.height;
        buffer_header_->width = cam_info.width;
        buffer_header_->binning_x = cam_info.binning_x;
        buffer_header_->binning_y = cam_info.binning_y;
        memcpy(buffer_header_->distortion_model, cam_info.distortion_model.c_str(), cam_info.distortion_model.size() + 1);
        buffer_header_->size_D = std::min<size_t>(cam_info.D.size(), 5); // Max 5
        memcpy(buffer_header_->D, cam_info.D.data(), buffer_header_->size_D*sizeof(double)); // std::vector
        memcpy(buffer_header_->K, &(cam_info.K.elems), 9*sizeof(double)); // boost::array
        memcpy(buffer_header_->R, &(cam_info.R.elems), 9*sizeof(double)); // boost::array
        memcpy(buffer_header_->P, &(cam_info.P.elems), 12*sizeof(double)); // boost::array
        // CameraInfo/roi
        buffer_header_->roi_x_offset = cam_info.roi.x_offset;
        buffer_header_->roi_y_offset = cam_info.roi.y_offset;
        buffer_header_->roi_height = cam_info.roi.height;
        buffer_header_->roi_width = cam_info.roi.width;
        buffer_header_->roi_do_rectify = cam_info.roi.do_rectify;
    }

    {
        ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header_->mutex);

        buffer_header_->timestamp = image.getTimestamp();

        memcpy(image_data_, rgb.data, rgb_data_size_);
        memcpy(image_data_ + rgb_data_size_, depth.data, depth_data_size_);

        buffer_header_->cond_empty.notify_one();
        ++buffer_header_->sequence_nr;
    }
}

// ----------------------------------------------------------------------------------------

void ServerSHM::checkSHMThreadFunc(const float frequency)
{
    ROS_DEBUG_NAMED("ServerSHM", "ServerSHM::checkSHMThreadFunc");
    ros::Rate r(frequency);
    while(nh_.ok())
    {
        ROS_DEBUG_STREAM_NAMED("ServerSHM", "ServerSHM::checkSHMThreadFunc: checking shm on: " << shared_mem_name_);
        try
        {
            ipc::shared_memory_object(ipc::open_only, shared_mem_name_.c_str(), ipc::read_only);
        }
        catch (ipc::interprocess_exception &ex)
        {
            ROS_FATAL_STREAM_NAMED("ServerSHM", "ServerSHM::checkSHMThreadFunc: SHM on '" << shared_mem_name_ << "' is corrupted: '" << ex.what() << "'");
            ros::shutdown();
        }
        r.sleep();
    }
    ROS_DEBUG_NAMED("ServerSHM", "ServerSHM::checkSHMThreadFunc done");
}

// ----------------------------------------------------------------------------------------

void pubHostnameThreadFunc(ros::NodeHandle& nh, const std::string server_name, const std::string hostname, const float frequency)
{
    ros::Publisher pub_shm_hostname = nh.advertise<std_msgs::String>(server_name + "/hosts", 1);
    ros::WallRate r(frequency);
    std_msgs::String msg;
    msg.data = hostname;
    while(nh.ok())
    {
        pub_shm_hostname.publish(msg);
        r.sleep();
    }
}

} // end namespace rgbd
