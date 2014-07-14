#ifndef RGBD_TRANSPORT_SERVER_H_
#define RGBD_TRANSPORT_SERVER_H_

#include <ros/ros.h>

namespace rgbd {

class Image;

class Server {

public:

    Server();

    virtual ~Server();

    void initialize(const std::string& name);

    void send(const Image& image);

    const static int SERIALIZATION_VERSION;

protected:

    ros::Publisher pub_image_;

};

}

#endif
