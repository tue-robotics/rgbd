#include "rgbd_transport/Client.h"

Client::Client() {

}

Client::~Client() {

}

void Client::intialize(const std::string& server_name) {
    ros::NodeHandle nh;
    sub_image_ = nh.subscribe(server_name, 1, &Client::imageCallback, this);
}

void Client::imageCallback(const rgbd_transport::RGBDMsg& msg) {
    std::cout << msg.depth.size() << std::endl;
}
