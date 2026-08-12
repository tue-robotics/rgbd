#include "rgbd/server_shm.h"
#include <algorithm>

#include "rgbd/image.h"
#include "rgbd/image_header.h"

#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/detail/os_file_functions.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <std_msgs/msg/string.hpp> // IWYU pragma: keep

namespace ipc = boost::interprocess;

namespace rgbd
{

ServerSHM::ServerSHM(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_server_shm"))
{
}

ServerSHM::~ServerSHM()
{
    stop_check_shm_thread_ = true;
    if (check_shm_thread_ptr_ && check_shm_thread_ptr_->joinable())
    {
        check_shm_thread_ptr_->join();
    }

    if (!shared_mem_name_.empty())
    {
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());
    }
}

void ServerSHM::initialize(const std::string& name)
{
    shared_mem_name_ = name;
    std::replace(shared_mem_name_.begin(), shared_mem_name_.end(), '/', '-');
}

void ServerSHM::send(const Image& image)
{
    if (shared_mem_name_.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ServerSHM"), "rgbd::SharedMemServer is not initialized");
        return;
    }

    const cv::Mat& rgb = image.getRGBImage();
    const cv::Mat& depth = image.getDepthImage();

    if (!buffer_header_)
    {
        ipc::shared_memory_object::remove(shared_mem_name_.c_str());
        shm_ = ipc::shared_memory_object(ipc::create_only, shared_mem_name_.c_str(), ipc::read_write);

        if (!check_shm_thread_ptr_)
        {
            check_shm_thread_ptr_ = std::make_unique<std::thread>(&ServerSHM::checkSHMThreadFunc, this, 1.0f);
        }

        rgb_data_size_ = static_cast<uint64_t>(rgb.cols) * rgb.rows * 3;
        depth_data_size_ = static_cast<uint64_t>(depth.cols) * depth.rows * 4;
        image_data_size_ = rgb_data_size_ + depth_data_size_;

        shm_.truncate(static_cast<ipc::offset_t>(sizeof(BufferHeader) + image_data_size_));

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

        const sensor_msgs::msg::CameraInfo& cam_info = image.getCameraModel().cameraInfo();
        buffer_header_->height = cam_info.height;
        buffer_header_->width = cam_info.width;
        buffer_header_->binning_x = cam_info.binning_x;
        buffer_header_->binning_y = cam_info.binning_y;
        memcpy(
            buffer_header_->distortion_model, cam_info.distortion_model.c_str(), cam_info.distortion_model.size() + 1);
        buffer_header_->size_D = std::min<size_t>(cam_info.d.size(), 5);
        memcpy(buffer_header_->D, cam_info.d.data(), buffer_header_->size_D * sizeof(double));
        memcpy(buffer_header_->K, cam_info.k.data(), 9 * sizeof(double));
        memcpy(buffer_header_->R, cam_info.r.data(), 9 * sizeof(double));
        memcpy(buffer_header_->P, cam_info.p.data(), 12 * sizeof(double));
        buffer_header_->roi_x_offset = cam_info.roi.x_offset;
        buffer_header_->roi_y_offset = cam_info.roi.y_offset;
        buffer_header_->roi_height = cam_info.roi.height;
        buffer_header_->roi_width = cam_info.roi.width;
        buffer_header_->roi_do_rectify = cam_info.roi.do_rectify;
    }

    {
        ipc::scoped_lock<ipc::interprocess_mutex> const lock(buffer_header_->mutex);

        buffer_header_->timestamp = image.getTimestamp();

        memcpy(image_data_, rgb.data, rgb_data_size_);
        memcpy(image_data_ + rgb_data_size_, depth.data, depth_data_size_);

        buffer_header_->cond_empty.notify_one();
        ++buffer_header_->sequence_nr;
    }
}

void ServerSHM::checkSHMThreadFunc(float frequency)
{
    rclcpp::Rate r(frequency);
    while (rclcpp::ok() && !stop_check_shm_thread_)
    {
        try
        {
            ipc::shared_memory_object(ipc::open_only, shared_mem_name_.c_str(), ipc::read_only);
        }
        catch (ipc::interprocess_exception& ex)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ServerSHM"),
                         "ServerSHM::checkSHMThreadFunc: SHM on '%s' is corrupted: '%s'",
                         shared_mem_name_.c_str(),
                         ex.what());
            rclcpp::shutdown();
            break;
        }
        r.sleep();
    }
}

void pubHostnameThreadFunc(const rclcpp::Node::SharedPtr& node,
                           const std::string& server_name,
                           const std::string& hostname,
                           float frequency)
{
    // std_msgs/msg/string.hpp is included; the detail struct header it suggests instead lacks the type-support
    // needed for create_publisher<T>.
    // NOLINTNEXTLINE(misc-include-cleaner)
    auto pub_shm_hostname = node->create_publisher<std_msgs::msg::String>(server_name + "/hosts", 1);
    rclcpp::WallRate r(frequency);
    std_msgs::msg::String msg;
    msg.data = hostname;
    while (rclcpp::ok())
    {
        pub_shm_hostname->publish(msg);
        r.sleep();
    }
}

} // namespace rgbd
