#include "rgbd_transport/Server.h"
#include "rgbd_transport/RGBDImage.h"

#include <opencv2/highgui/highgui.hpp>

Server::Server() {

}

Server::~Server() {

}

void Server::initialize(const std::string& name) {
}

void Server::send(const RGBDImage& image) {
    std::cout << "Sending with frame " << image.getFrameID() << std::endl;

    cv::imshow("rgb", image.getRGBImage());
    cv::imshow("depth", image.getDepthImage() / 8);
    cv::waitKey(3);
}
