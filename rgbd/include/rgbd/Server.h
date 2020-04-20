#ifndef RGBD_SERVER_H_
#define RGBD_SERVER_H_

#include "rgbd/Image.h"
#include "rgbd/Server_RGBD.h"
#include "rgbd/Server_SHM.h"

namespace rgbd {

class Image;

/**
 * @brief Server which provides interfaces of ServerRGBD and ServerSHM
 */
class Server {

public:

    /**
     * @brief Constructor
     */
    Server();

    /**
     * @brief Destructor
     */
    virtual ~Server();

    /**
     * @brief initialize initialize server
     * @param name Fully resolved server name
     * @param rgb_type rgb storage type
     * @param depth_type depth storage type
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS);

    /**
     * @brief send Write a new image to all interfaces
     * @param image Image to be written
     * @param threaded Use any threaded version of send of the interfaces
     */
    void send(const Image& image, bool threaded=false);

protected:

    ServerRGBD server_rgbd_;

    ServerSHM server_shm_;

};

}

#endif // RGBD_SERVER_H_
