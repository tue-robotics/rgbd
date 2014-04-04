#include "rgbd_transport/Client.h"
#include <opencv2/highgui/highgui.hpp>

namespace rgbd {

Client::Client() {
}

Client::~Client(){
}

void Client::intialize(const std::string& server_name) {
    ros::NodeHandle nh;

    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<rgbd_transport::RGBDMsg>(
                server_name, 1, boost::bind(&Client::imageCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_image_ = nh.subscribe(sub_options);
}

bool Client::nextImage(RGBDImage& image) {
    received_image_ = false;
    image_ptr_ = &image;
    cb_queue_.callAvailable();
    return received_image_;
}

void Client::imageCallback(const rgbd_transport::RGBDMsg::ConstPtr& msg) {

    float depthQuantA = msg->params[0];
    float depthQuantB = msg->params[1];

    cv::Mat decompressed = cv::imdecode(msg->depth, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth_image(decompressed.size(), CV_32FC1);

    // Depth conversion
    cv::MatIterator_<float> itDepthImg = depth_image.begin<float>(),
            itDepthImg_end = depth_image.end<float>();
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

    image_ptr_->setDepthImage(depth_image);

    // DESERIALZE CAMERA INFO
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
    cam_info_msg.width = depth_image.cols;
    cam_info_msg.height = depth_image.rows;

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info_msg);

    image_ptr_->setCameraModel(cam_model);
    image_ptr_->setFrameID(msg->header.frame_id);
    image_ptr_->setTimestamp(msg->header.stamp.toSec());

    received_image_ = true;
}

}
