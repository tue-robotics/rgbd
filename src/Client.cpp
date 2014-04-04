#include "rgbd_transport/Client.h"
#include <opencv2/highgui/highgui.hpp>

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
    received_image_ = true;
}
