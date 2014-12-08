#include "rgbd/Server.h"
#include "rgbd/RGBDMsg.h"
#include "rgbd/Image.h"
#include "rgbd/serialization.h"

#include <opencv2/highgui/highgui.hpp>

#include <tue/serialization/conversions.h>

#include <tue/profiling/timer.h>

namespace rgbd {

const int Server::SERIALIZATION_VERSION = 2;

// ----------------------------------------------------------------------------------------

Server::Server() {
}

// ----------------------------------------------------------------------------------------

Server::~Server() {
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type) {
    ros::NodeHandle nh;
    pub_image_ = nh.advertise<rgbd::RGBDMsg>(name, 1);
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image) {
    rgbd::RGBDMsg msg;
    msg.version = SERIALIZATION_VERSION;

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    serialize(image, a, rgb_type_, depth_type_);
    tue::serialization::convert(stream, msg.rgb);
    pub_image_.publish(msg);
}

}
