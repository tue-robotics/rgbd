#include "rgbd/client_shm.h"
#include <algorithm>

#include "rgbd/image.h"
#include "rgbd/image_header.h"
#include "rgbd/types.h"

#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/detail/os_file_functions.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <cstdint>
#include <cstring>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rclcpp/utilities.hpp>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <chrono>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <string>

namespace ipc = boost::interprocess;

namespace rgbd
{

ClientSHM::ClientSHM() = default;

ClientSHM::~ClientSHM() = default;

bool ClientSHM::initialize(const std::string& server_name, float timeout)
{
    std::string server_name_cp = server_name;
    std::replace(server_name_cp.begin(), server_name_cp.end(), '/', '-');

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
                               static_cast<double>(timeout))
    {
        try
        {
            shm_ = ipc::shared_memory_object(ipc::open_only, server_name_cp.c_str(), ipc::read_write);

            mem_buffer_header_ = ipc::mapped_region(shm_, ipc::read_write, 0, sizeof(BufferHeader));
            mem_image_ = ipc::mapped_region(shm_, ipc::read_only, sizeof(BufferHeader));

            buffer_header_ = static_cast<BufferHeader*>(mem_buffer_header_.get_address());
            image_data_ = static_cast<uchar*>(mem_image_.get_address());

            rgb_data_size_ = static_cast<uint64_t>(buffer_header_->rgb_width) * buffer_header_->rgb_height * 3;
            depth_data_size_ = static_cast<uint64_t>(buffer_header_->depth_width) * buffer_header_->depth_height * 4;

            sequence_nr_ = 0;
            RCLCPP_INFO(
                rclcpp::get_logger("ClientSHM"), "Opened shared memory on: '%s' successfully.", server_name_cp.c_str());
            return true;
        }
        catch (ipc::interprocess_exception& ex)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("ClientSHM"), "Could not open shared memory: %s", ex.what());
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(rclcpp::get_logger("ClientSHM"),
                "Opening shared memory on: '%s' failed on timeout(%f).",
                server_name_cp.c_str(),
                timeout);

    return false;
}

bool ClientSHM::deinitialize()
{
    buffer_header_ = nullptr;
    image_data_ = nullptr;
    mem_image_ = ipc::mapped_region();
    mem_buffer_header_ = ipc::mapped_region();
    shm_ = ipc::shared_memory_object();
    return true;
}

bool ClientSHM::nextImage(Image& image)
{
    if (!initialized())
    {
        return false;
    }

    ipc::scoped_lock<ipc::interprocess_mutex> const lock(buffer_header_->mutex);

    if (buffer_header_->sequence_nr == sequence_nr_)
    {
        return false;
    }

    cv::Mat* rgb = &(image.rgb_image_);
    cv::Mat* depth = &(image.depth_image_);

    *rgb = cv::Mat(buffer_header_->rgb_height, buffer_header_->rgb_width, CV_8UC3);
    *depth = cv::Mat(buffer_header_->depth_height, buffer_header_->depth_width, CV_32FC1);

    memcpy(rgb->data, image_data_, rgb_data_size_);
    memcpy(depth->data, image_data_ + rgb_data_size_, depth_data_size_);

    image.frame_id_ = buffer_header_->frame_id;
    image.timestamp_ = buffer_header_->timestamp;

    if (!image.getCameraModel().initialized())
    {
        sensor_msgs::msg::CameraInfo cam_info_msg;

        cam_info_msg.height = buffer_header_->height;
        cam_info_msg.width = buffer_header_->width;
        cam_info_msg.binning_x = buffer_header_->binning_x;
        cam_info_msg.binning_y = buffer_header_->binning_y;
        cam_info_msg.distortion_model = buffer_header_->distortion_model;
        cam_info_msg.d.resize(buffer_header_->size_D);
        memcpy(cam_info_msg.d.data(), buffer_header_->D, buffer_header_->size_D * sizeof(double));
        memcpy(cam_info_msg.k.data(), buffer_header_->K, 9 * sizeof(double));
        memcpy(cam_info_msg.r.data(), buffer_header_->R, 9 * sizeof(double));
        memcpy(cam_info_msg.p.data(), buffer_header_->P, 12 * sizeof(double));
        cam_info_msg.roi.x_offset = buffer_header_->roi_x_offset;
        cam_info_msg.roi.y_offset = buffer_header_->roi_y_offset;
        cam_info_msg.roi.height = buffer_header_->roi_height;
        cam_info_msg.roi.width = buffer_header_->roi_width;
        cam_info_msg.roi.do_rectify = buffer_header_->roi_do_rectify;

        image.cam_model_.fromCameraInfo(cam_info_msg);
    }

    image.setFrameId(buffer_header_->frame_id);
    image.setTimestamp(buffer_header_->timestamp);

    sequence_nr_ = buffer_header_->sequence_nr;

    return true;
}

ImagePtr ClientSHM::nextImage()
{
    ImagePtr img(new Image);
    if (nextImage(*img))
    {
        return img;
    }
    return nullptr;
}

} // namespace rgbd
