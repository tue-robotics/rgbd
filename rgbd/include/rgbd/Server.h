#ifndef RGBD_TRANSPORT_SERVER_H_
#define RGBD_TRANSPORT_SERVER_H_

#include "rgbd/Image.h"
#include "rgbd/Server_RGBD.h"
#include "rgbd/Server_SHM.h"

#include "rgbd_msgs/GetRGBD.h"

#include <ros/publisher.h>
#include <ros/callback_queue.h>
#include <ros/service_server.h>

#include <boost/thread.hpp>

namespace rgbd {

class Image;

class Server {

public:

    Server();

    virtual ~Server();

    /**
     * @brief initialize initialize server
     * @param name Fully resolved server name
     * @param rgb_type
     * @param depth_type
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS);

    void send(const Image& image, bool threaded=false);

protected:

    rgbd::Image image_;

    ServerRGBD server_rgbd_;

    ServerSHM server_shm_;


};

}

#endif
