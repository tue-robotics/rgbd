#ifndef RGBD_TRANSPORT_CLIENT_H_
#define RGBD_TRANSPORT_CLIENT_H_

#include <ros/ros.h>

#include "rgbd/Image.h"
#include "rgbd/RGBDMsg.h"
#include <ros/callback_queue.h>

#include "rgbd/types.h"

namespace rgbd {

class Client {

public:

    Client();

    virtual ~Client();

    void intialize(const std::string& server_name);

    bool initialized() { return !sub_image_.getTopic().empty(); }

    bool nextImage(Image& image);

    ImagePtr nextImage();

protected:

    ros::Subscriber sub_image_;
    ros::CallbackQueue cb_queue_;

    bool received_image_;
    Image* image_ptr_;

    void imageCallback(const rgbd::RGBDMsg::ConstPtr& msg);

};

}

#endif
