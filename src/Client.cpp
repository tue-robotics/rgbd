#include "rgbd/Client.h"
#include <opencv2/highgui/highgui.hpp>
#include "rgbd/Image.h"

// RGBD message serialization
#include "rgbd/serialization.h"
#include <tue/serialization/conversions.h>

// ROS message serialization
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectApproxPolicy;

namespace rgbd {

// ----------------------------------------------------------------------------------------

struct ROSImageSyncData
{
    ROSImageSyncData() : sync_(0), sub_rgb_sync_(0), sub_depth_sync_(0) {}

    ~ROSImageSyncData()
    {
        sub_cam_info_.shutdown();
        delete sub_rgb_sync_;
        delete sub_depth_sync_;
        delete sync_;
    }

    message_filters::Synchronizer<KinectApproxPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_rgb_sync_;
    message_filters::Subscriber<sensor_msgs::Image>* sub_depth_sync_;
    ros::Subscriber sub_cam_info_;
    image_geometry::PinholeCameraModel cam_model_;
};

// ----------------------------------------------------------------------------------------

Client::Client() : nh_(0), ros_image_sync_data_(0)
{
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
    delete nh_;
    delete ros_image_sync_data_;
    sub_image_.shutdown();
}

// ----------------------------------------------------------------------------------------

void Client::intialize(const std::string& server_name)
{
    if (shared_mem_client_.intialize(server_name))
        return;

    // If the shared memory client could not be created, use ROS topics instead

    delete nh_;
    nh_ = new ros::NodeHandle();

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd::RGBDMsg>(
                server_name, 1, boost::bind(&Client::rgbdImageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh_->subscribe(sub_options);
}

// ----------------------------------------------------------------------------------------

void Client::intialize(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
    delete nh_;
    nh_ = new ros::NodeHandle();
    nh_->setCallbackQueue(&cb_queue_);

    ros_image_sync_data_ = new ROSImageSyncData();

    ros_image_sync_data_->sub_cam_info_ = nh_->subscribe(cam_info_topic, 1, &Client::camInfoCallback, this);

    ros_image_sync_data_->sub_rgb_sync_ = new message_filters::Subscriber<sensor_msgs::Image>(*nh_, rgb_image_topic, 1);
    ros_image_sync_data_->sub_depth_sync_ = new message_filters::Subscriber<sensor_msgs::Image>(*nh_, depth_image_topic, 1);

    ros_image_sync_data_->sync_ = new message_filters::Synchronizer<KinectApproxPolicy>(KinectApproxPolicy(10), *ros_image_sync_data_->sub_rgb_sync_, *ros_image_sync_data_->sub_depth_sync_);
    ros_image_sync_data_->sync_->registerCallback(boost::bind(&Client::imageCallback, this, _1, _2));
}

// ----------------------------------------------------------------------------------------

bool Client::nextImage(Image& image)
{
    if (shared_mem_client_.initialized())
        return shared_mem_client_.nextImage(image);

    received_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return received_image_;
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage() {
    if (shared_mem_client_.initialized())
    {
        ImagePtr img(new Image); // TODO
        if (shared_mem_client_.nextImage(*img))
            return img;
        else
            return ImagePtr();
    }

    image_ptr_ = 0;
    cb_queue_.callAvailable();
    return ImagePtr(image_ptr_);
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

void Client::imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg)
{
    if (!ros_image_sync_data_->cam_model_.initialized())
        return;

    cv_bridge::CvImagePtr img_ptr, depth_img_ptr;

    // Convert RGB image
    try
    {
        img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not deserialize rgb image: %s", e.what());
        return;
    }

    // Convert depth image
    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, depth_image_msg->encoding);

        if (depth_image_msg->encoding == "16UC1")
        {
            // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
            cv::Mat depth_image(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1);
            for(int x = 0; x < depth_image.cols; ++x)
            {
                for(int y = 0; y < depth_image.rows; ++y)
                {
                    depth_image.at<float>(y, x) = ((float)depth_img_ptr->image.at<unsigned short>(y, x)) / 1000; // (mm to m)
                }
            }

            depth_img_ptr->image = depth_image;
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not deserialize depth image: %s", e.what());
        return;
    }

    if (!image_ptr_) {
        // in this case, the pointer will always be wrapped in a shared ptr, so no mem leaks (see nextImage() )
        image_ptr_ = new Image();
    }

    image_ptr_->rgb_image_ = img_ptr->image;
    image_ptr_->depth_image_ = depth_img_ptr->image;
    image_ptr_->frame_id_ = rgb_image_msg->header.frame_id;
    image_ptr_->timestamp_ = rgb_image_msg->header.stamp.toSec();
}

// ----------------------------------------------------------------------------------------

void Client::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    if (!ros_image_sync_data_->cam_model_.initialized())
        ros_image_sync_data_->cam_model_.fromCameraInfo(cam_info_msg);
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

void Client::rgbdImageCallback(const rgbd::RGBDMsg::ConstPtr& msg) {

    if (msg->version == 0)
    {
        std::cout << "rgbd_transport::Client::imageCallback: version 0 not supported" << std::endl;
        return;
    }

    if (!image_ptr_) {
        // in this case, the pointer will always be wrapped in a shared ptr, so no mem leaks (see nextImage() )
        image_ptr_ = new Image();
    }

    if (msg->version == 1)
    {
        // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

        image_ptr_->rgb_image_ = cv::imdecode(cv::Mat(msg->rgb), CV_LOAD_IMAGE_UNCHANGED);

        // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

        float depthQuantA = msg->params[0];
        float depthQuantB = msg->params[1];

        cv::Mat decompressed = cv::imdecode(msg->depth, CV_LOAD_IMAGE_UNCHANGED);
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

        received_image_ = true;
    }
    else if (msg->version == 2)
    {
        std::stringstream stream;
        tue::serialization::convert(msg->rgb, stream);
        tue::serialization::InputArchive a(stream);
        rgbd::deserialize(a, *image_ptr_);
        received_image_ = true;
    }
}

}
