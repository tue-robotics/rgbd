#include "rgbd/Server.h"
#include "rgbd/RGBDMsg.h"
#include "rgbd/RGBDImage.h"

#include <opencv2/highgui/highgui.hpp>

namespace rgbd {

const int Server::SERIALIZATION_VERSION = 1;

// ----------------------------------------------------------------------------------------

Server::Server() {
}

// ----------------------------------------------------------------------------------------

Server::~Server() {
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name) {
    ros::NodeHandle nh;
    pub_image_ = nh.advertise<rgbd::RGBDMsg>(name, 1);
}

// ----------------------------------------------------------------------------------------

void Server::send(const RGBDImage& image) {
    rgbd::RGBDMsg msg;

    // - - - - - - - - - - - - - - - - GENERAL INFO - - - - - - - - - - - - - - - -

    msg.version = SERIALIZATION_VERSION;
    msg.header.frame_id = image.getFrameId();
    msg.header.stamp = ros::Time(image.getTimestamp());

    // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

    const image_geometry::PinholeCameraModel& cam_model = image.getCameraModel();
    msg.cam_info.push_back(cam_model.fx());
    msg.cam_info.push_back(cam_model.fy());
    msg.cam_info.push_back(cam_model.cx());
    msg.cam_info.push_back(cam_model.cy());
    msg.cam_info.push_back(cam_model.Tx());
    msg.cam_info.push_back(cam_model.Ty());

    // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

    // OpenCV compression settings
    std::vector<int> rgb_params;
    rgb_params.resize(3, 0);

    rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
    rgb_params[1] = 95; // default is 95

    // Compress image
    if (!cv::imencode(".jpg", image.rgb_image_, msg.rgb, rgb_params)) {
        std::cout << "RGB image compression failed" << std::endl;
        return;
    }

    // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

    float depthZ0 = 100; //config_.depth_quantization;
    float depthMax = 10; //config_.depth_max;

    float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
    float depthQuantB = 1.0f - depthQuantA / depthMax;
    msg.params.push_back(depthQuantA);
    msg.params.push_back(depthQuantB);


    const cv::Mat& depth_image = image.depth_image_;
    cv::Mat invDepthImg(depth_image.size(), CV_16UC1);

    // Matrix iterators
    cv::MatConstIterator_<float> itDepthImg = depth_image.begin<float>(),
                             itDepthImg_end = depth_image.end<float>();
    cv::MatIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>(),
                                 itInvDepthImg_end = invDepthImg.end<unsigned short>();

    // Quantization
    for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
        // check for NaN & max depth
        if (*itDepthImg < depthMax){
            *itInvDepthImg = depthQuantA / *itDepthImg + depthQuantB;
        } else{
            *itInvDepthImg = 0;
        }
    }

    // Compression settings
    std::vector<int> params;
    params.resize(3, 0);

    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = 1;

    if (cv::imencode(".png", invDepthImg, msg.depth, params)) {
        pub_image_.publish(msg);
    } else {
        std::cout << "Depth image compression failed" << std::endl;
    }
}

}
