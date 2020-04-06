#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>

#include "rgbd/Client.h"
#include "rgbd/Image.h"

// RGBD message serialization
#include "rgbd/serialization.h"
#include <tue/serialization/conversions.h>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client() : nh_(nullptr)
{
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
    sub_image_.shutdown();
    delete nh_;
}

// ----------------------------------------------------------------------------------------

void Client::intialize(const std::string& server_name, float timeout)
{
    if (shared_mem_client_.intialize(server_name, timeout))
        return;

    // If the shared memory client could not be created, use ROS topics instead

    delete nh_;
    nh_ = new ros::NodeHandle();

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd_msgs::RGBD>(
                server_name, 1, boost::bind(&Client::rgbdImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh_->subscribe(sub_options);
}

// ----------------------------------------------------------------------------------------

bool Client::nextImage(Image& image)
{
    if (shared_mem_client_.initialized())
        return shared_mem_client_.nextImage(image);

    new_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return new_image_;
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage()
{
    if (shared_mem_client_.initialized())
    {
        ImagePtr img(new Image);
        if (shared_mem_client_.nextImage(*img))
            return img;
        else
            return ImagePtr();
    }

    image_ptr_ = nullptr;
    cb_queue_.callAvailable();
    return ImagePtr(image_ptr_);
}

// ----------------------------------------------------------------------------------------

void Client::rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg) {

    if (msg->version == 0)
    {
        std::cout << "rgbd_transport::Client::imageCallback: version 0 not supported" << std::endl;
        return;
    }

    if (!image_ptr_)
        // in this case, the pointer will always be wrapped in a shared ptr, so no mem leaks (see nextImage() )
        image_ptr_ = new Image();

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

        image_ptr_->cam_model_.fromCameraInfo(cam_info_msg);
        image_ptr_->timestamp_ = msg->header.stamp.toSec();
        image_ptr_->frame_id_ = msg->header.frame_id;

        new_image_ = true;
    }
    else if (msg->version == 2)
    {
        std::stringstream stream;
        tue::serialization::convert(msg->rgb, stream);
        tue::serialization::InputArchive a(stream);
        rgbd::deserialize(a, *image_ptr_);
        new_image_ = true;
    }
}

}
