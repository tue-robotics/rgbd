#ifndef RGBD_TRANSPORT_CLIENT_H_
#define RGBD_TRANSPORT_CLIENT_H_

#include <ros/ros.h>

#include "rgbd_transport/RGBDMsg.h"

class Client {

public:

    Client();

    virtual ~Client();

    void intialize(const std::string& server_name);

protected:

    ros::Subscriber sub_image_;

    void imageCallback(const rgbd_transport::RGBDMsg& msg);

};

#endif
