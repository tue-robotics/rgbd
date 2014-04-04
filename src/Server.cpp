#include "rgbd_transport/Server.h"
#include "rgbd_transport/RGBDMsg.h"
#include "rgbd_transport/RGBDImage.h"

#include <opencv2/highgui/highgui.hpp>

const int Server::SERIALIZATION_VERSION = 0;

Server::Server() {
}

Server::~Server() {
}

void Server::initialize(const std::string& name) {
    ros::NodeHandle nh;
    pub_image_ = nh.advertise<rgbd_transport::RGBDMsg>(name, 1);
}

void Server::send(const RGBDImage& image) {
    rgbd_transport::RGBDMsg msg;
    msg.version = SERIALIZATION_VERSION;
    msg.header.frame_id = image.getFrameID();
    msg.header.stamp = ros::Time(image.getTimestamp());

    // Compression settings
    std::vector<int> params;
    params.resize(3, 0);

    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = 2;

    cv::Mat invDepthImg = image.getDepthImage();

    if (cv::imencode(".png", invDepthImg, msg.depth, params)) {
        std::cout << msg.depth.size() << std::endl;
    } else {
        std::cout << "failed" << std::endl;
    }

    pub_image_.publish(msg);
}
