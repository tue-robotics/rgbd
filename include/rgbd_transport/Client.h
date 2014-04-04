#ifndef RGBD_TRANSPORT_CLIENT_H_
#define RGBD_TRANSPORT_CLIENT_H_

#include <ros/ros.h>

#include "rgbd_transport/RGBDImage.h"
#include "rgbd_transport/RGBDMsg.h"
#include <ros/callback_queue.h>

namespace rgbd {

class Client {

public:

    Client();

    virtual ~Client();

    void intialize(const std::string& server_name);

    bool nextImage(RGBDImage& image);

protected:

    ros::Subscriber sub_image_;
    ros::CallbackQueue cb_queue_;

    bool received_image_;
    RGBDImage* image_ptr_;

    void imageCallback(const rgbd_transport::RGBDMsg::ConstPtr& msg);

};

}

#endif
