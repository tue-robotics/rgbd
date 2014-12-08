#ifndef RGBD_TRANSPORT_SERVER_H_
#define RGBD_TRANSPORT_SERVER_H_

#include "rgbd/Image.h"

#include <ros/ros.h>

namespace rgbd {

class Image;

class Server {

public:

    Server();

    virtual ~Server();

    void initialize(const std::string& name, RGBStorageType rgb_type, DepthStorageType depth_type);

    void send(const Image& image);

    const static int SERIALIZATION_VERSION;

protected:

    ros::Publisher pub_image_;
    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

};

}

#endif
