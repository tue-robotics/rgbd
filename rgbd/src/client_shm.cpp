#include "rgbd/client_shm.h"

#include "rgbd/image.h"

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/console.h>
#include <ros/time.h>

#include <sensor_msgs/CameraInfo.h>

#include <boost/interprocess/sync/scoped_lock.hpp>

namespace ipc = boost::interprocess;

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

ClientSHM::ClientSHM() : buffer_header_(nullptr), image_data_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

ClientSHM::~ClientSHM()
{
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::initialize(const std::string& server_name, float timeout)
{
    std::string server_name_cp = server_name;
    std::replace(server_name_cp.begin(), server_name_cp.end(), '/', '-');

    ros::Time start = ros::Time::now();
    ros::Time now;
    ros::WallDuration d(0.1);
    do
    {
        try
        {
            // Open already created shared memory object.
            shm_ = ipc::shared_memory_object(ipc::open_only, server_name_cp.c_str(), ipc::read_write);

            mem_buffer_header_ = ipc::mapped_region(shm_, ipc::read_write, 0, sizeof(BufferHeader));
            mem_image_ = ipc::mapped_region(shm_, ipc::read_only, sizeof(BufferHeader));

            buffer_header_ = static_cast<BufferHeader*>(mem_buffer_header_.get_address());
            image_data_ = static_cast<uchar*>(mem_image_.get_address());

            rgb_data_size_ = static_cast<uint64_t>(buffer_header_->rgb_width * buffer_header_->rgb_height * 3);
            depth_data_size_ = static_cast<uint64_t>(buffer_header_->depth_width * buffer_header_->depth_height * 4);

            sequence_nr_ = 0;
            ROS_INFO_STREAM_NAMED("ClientSHM", "Opened shared memory on: '" << server_name_cp << "' succesfully.");
            return true;
        }
        catch(ipc::interprocess_exception &ex)
        {
            ROS_DEBUG_STREAM_NAMED("ClientSHM", "Could not open shared memory: " << ex.what());
        }
        d.sleep();
        now = ros::Time::now();
    }
    while (ros::ok() && (now - start).toSec() < static_cast<double>(timeout));

    ROS_INFO_STREAM_NAMED("ClientSHM", "Opening shared memory on: '" << server_name_cp << "' failed on timeout(" << timeout << ").");

    return false;
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::deinitialize()
{
    buffer_header_ = nullptr;
    image_data_ = nullptr;
    mem_image_ = ipc::mapped_region();
    mem_buffer_header_ = ipc::mapped_region();
    shm_= ipc::shared_memory_object();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::nextImage(Image& image)
{
    if (!initialized())
        return false;

    ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header_->mutex);

    if (buffer_header_->sequence_nr == sequence_nr_)
        return false;

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
        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.height = buffer_header_->height;
        cam_info_msg.width = buffer_header_->width;
        cam_info_msg.binning_x = buffer_header_->binning_x;
        cam_info_msg.binning_y = buffer_header_->binning_y;
        cam_info_msg.distortion_model = buffer_header_->distortion_model;
        cam_info_msg.D.resize(buffer_header_->size_D); // std::vector
        memcpy(cam_info_msg.D.data(), buffer_header_->D, buffer_header_->size_D*sizeof(double)); // std::vector
        memcpy(&(cam_info_msg.K.elems), buffer_header_->K, 9*sizeof(double)); // boost::array
        memcpy(&(cam_info_msg.R.elems), buffer_header_->R, 9*sizeof(double)); // boost::array
        memcpy(&(cam_info_msg.P.elems), buffer_header_->P, 12*sizeof(double)); // boost::array
        // CameraInfo/roi
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

// ----------------------------------------------------------------------------------------------------

ImagePtr ClientSHM::nextImage()
{
    ImagePtr img(new Image);
    if (nextImage(*img))
        return img;
    else
        return nullptr;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace rgbd

