#include "rgbd/ServerShmOnly.h"
#include "rgbd_msgs/RGBD.h"
#include "rgbd/Image.h"
#include "rgbd/serialization.h"

#include <opencv2/highgui/highgui.hpp>

#include <tue/serialization/conversions.h>

#include <ros/node_handle.h>

namespace rgbd {

const int ServerShmOnly::SERIALIZATION_VERSION = 2;

// ----------------------------------------------------------------------------------------

ServerShmOnly::ServerShmOnly()
{
}

// ----------------------------------------------------------------------------------------

ServerShmOnly::~ServerShmOnly()
{
    send_thread_.join();
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type)
{
    ros::NodeHandle nh;
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;

    // Initialize shared mem server
    shared_mem_server_.initialize(name);
}

void ServerShmOnly::rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg)
{
    if (msg->version == 0)
    {
        std::cout << "rgbdImageCallback: version 0 not supported" << std::endl;
        return;
    }

    if (!image_ptr_)
        image_ptr_.reset(new rgbd::Image);

    if (msg->version == 1)
    {
        // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

        image_ptr_->rgb_image_ = cv::imdecode(cv::Mat(msg->rgb), cv::IMREAD_UNCHANGED);

        // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

        float depthQuantA = msg->params[0];
        float depthQuantB = msg->params[1];

        cv::Mat decompressed = cv::imdecode(msg->depth, cv::IMREAD_UNCHANGED);
        image_ptr_->depth_image_ = cv::Mat(decompressed.size(), CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> itDepthImg = image_ptr_->depth_image_.begin<float>(),
                itDepthImg_end = image_ptr_->depth_image_.end<float>();
        cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
            // check for NaN & max depth
            if (*itInvDepthImg) {
                *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
            } else{
                *itDepthImg = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.D.resize(5, 0.0);
        cam_info_msg.K.fill(0.0);
        cam_info_msg.K[0] = msg->cam_info[0];  // fx
        cam_info_msg.K[2] = msg->cam_info[2];  // cx
        cam_info_msg.K[4] = msg->cam_info[1];  // fy
        cam_info_msg.K[5] = msg->cam_info[3];  // cy
        cam_info_msg.K[8] = 1.0;

        cam_info_msg.R.fill(0.0);
        cam_info_msg.R[0] = 1.0;
        cam_info_msg.R[4] = 1.0;
        cam_info_msg.R[8] = 1.0;

        cam_info_msg.P.fill(0.0);
        cam_info_msg.P[0] = msg->cam_info[0];  // fx
        cam_info_msg.P[2] = msg->cam_info[2];  // cx
        cam_info_msg.P[3] = msg->cam_info[4];  // Tx
        cam_info_msg.P[5] = msg->cam_info[1];  // fy
        cam_info_msg.P[6] = msg->cam_info[3];  // cy
        cam_info_msg.P[7] = msg->cam_info[5];  // cy
        cam_info_msg.P[10] = 1.0;

        cam_info_msg.distortion_model = "plumb_bob";
        cam_info_msg.width = image_ptr_->rgb_image_.cols;
        cam_info_msg.height = image_ptr_->rgb_image_.rows;

        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info_msg);

        image_ptr_->cam_model_ = cam_model;
        image_ptr_->timestamp_ = msg->header.stamp.toSec();
        image_ptr_->frame_id_ = msg->header.frame_id;

        image_ptr_->setupRasterizer();
    }
    else if (msg->version == 2)
    {
        std::stringstream stream;
        tue::serialization::convert(msg->rgb, stream);
        tue::serialization::InputArchive a(stream);
        rgbd::deserialize(a, *image_ptr_);
    }

    send(*image_ptr_);
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::send(const Image& image, bool threaded)
{
    if (!threaded)
    {
        sendImpl(image);
        return;
    }

    send_thread_ = boost::thread(&ServerShmOnly::sendImpl, this, image);
}

// ----------------------------------------------------------------------------------------

void ServerShmOnly::sendImpl(const Image& image)
{    
    // Send image using shared memory
    boost::mutex::scoped_lock lock(send_mutex_shared_, boost::try_to_lock);
    if (!lock)
        return;

    shared_mem_server_.send(image);
}

}
