#include "rgbd/Client_SHM.h"

#include "rgbd/Image.h"

#include <ros/init.h>
#include <ros/time.h>

#include <sensor_msgs/CameraInfo.h>

#include <boost/interprocess/sync/scoped_lock.hpp>

namespace ipc = boost::interprocess;

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

ClientSHM::ClientSHM() : buffer_header(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

ClientSHM::~ClientSHM()
{
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::intialize(const std::string& server_name, float timeout)
{
    ros::Time start = ros::Time::now();
    ros::Time now;
    ros::Duration d(0.1);
    do
    {
        try
        {
            std::string server_name_cp = server_name;
            std::replace(server_name_cp.begin(), server_name_cp.end(), '/', '-');

            // Open already created shared memory object.
            shm = ipc::shared_memory_object(ipc::open_only, server_name_cp.c_str(), ipc::read_write);

            mem_buffer_header = ipc::mapped_region(shm, ipc::read_write, 0, sizeof(BufferHeader));
            mem_image = ipc::mapped_region(shm, ipc::read_only, sizeof(BufferHeader));

            buffer_header = static_cast<BufferHeader*>(mem_buffer_header.get_address());

            sequence_nr = 0;
            return true;
        }
        catch(ipc::interprocess_exception &ex)
        {
//            std::cout << ex.what() << std::endl;
        }
        d.sleep();
        now = ros::Time::now();
     }
     while (ros::ok() && (now - start).toSec() < timeout);

    return false;
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::initialized()
{
    return (buffer_header != nullptr);
}

// ----------------------------------------------------------------------------------------------------

bool ClientSHM::nextImage(Image& image)
{
    if (!initialized())
        return false;

    ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header->mutex);

    if (buffer_header->sequence_nr == sequence_nr)
        return false;

    cv::Mat* rgb = &(image.rgb_image_);
    cv::Mat* depth = &(image.depth_image_);

    //    if (buffer_header.sequence_nr == sequence_nr)
    //        buffer_header.cond_empty.wait(lock);

    uchar* image_data = static_cast<uchar*>(mem_image.get_address());

    uint64_t rgb_data_size = buffer_header->rgb_width * buffer_header->rgb_height * 3;
    uint64_t depth_data_size = buffer_header->depth_width * buffer_header->depth_height * 4;

    *rgb = cv::Mat(buffer_header->rgb_height, buffer_header->rgb_width, CV_8UC3);
    *depth = cv::Mat(buffer_header->depth_height, buffer_header->depth_width, CV_32FC1);

    memcpy(rgb->data, image_data, rgb_data_size);
    memcpy(depth->data, image_data + rgb_data_size, depth_data_size);


    if (!image.cam_model_.initialized())
    {
        sensor_msgs::CameraInfo cam_info_msg;
        cam_info_msg.header.frame_id = buffer_header->frame_id;
        cam_info_msg.header.stamp.fromSec(buffer_header->timestamp);

        cam_info_msg.height = buffer_header->height;
        cam_info_msg.width = buffer_header->width;
        cam_info_msg.binning_x = buffer_header->binning_x;
        cam_info_msg.binning_y = buffer_header->binning_y;
        cam_info_msg.distortion_model = buffer_header->distortion_model;
        cam_info_msg.D.resize(buffer_header->size_D); // std::vector
        memcpy(cam_info_msg.D.data(), buffer_header->D, buffer_header->size_D*sizeof(double)); // std::vector
        memcpy(&(cam_info_msg.K.elems), buffer_header->K, 9*sizeof(double)); // boost::array
        memcpy(&(cam_info_msg.R.elems), buffer_header->R, 9*sizeof(double)); // boost::array
        memcpy(&(cam_info_msg.P.elems), buffer_header->P, 12*sizeof(double)); // boost::array
        // CameraInfo/roi
        cam_info_msg.roi.x_offset = buffer_header->roi_x_offset;
        cam_info_msg.roi.y_offset = buffer_header->roi_y_offset;
        cam_info_msg.roi.height = buffer_header->roi_height;
        cam_info_msg.roi.width = buffer_header->roi_width;
        cam_info_msg.roi.do_rectify = buffer_header->roi_do_rectify;

        image.cam_model_.fromCameraInfo(cam_info_msg);
    }

    image.frame_id_ = buffer_header->frame_id;
    image.timestamp_ = buffer_header->timestamp;

    sequence_nr = buffer_header->sequence_nr;

    return true;
}

} // end namespace rgbd
