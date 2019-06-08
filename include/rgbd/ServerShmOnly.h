#ifndef SERVER_SHM_ONLY_H_
#define SERVER_SHM_ONLY_H_

#include "rgbd/Image.h"
#include "rgbd/shared_mem_server.h"

#include "rgbd/RGBDMsg.h"

#include <boost/thread.hpp>

namespace rgbd {

class Image;

class ServerShmOnly {

public:

    ServerShmOnly();

    virtual ~ServerShmOnly();

    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS);

    void rgbdImageCallback(const RGBDMsg::ConstPtr& msg);

    void send(const Image& image, bool threaded = false);

    const static int SERIALIZATION_VERSION;

protected:

    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

    ImagePtr image_ptr_;

    SharedMemServer shared_mem_server_;

    // Threaded sending
    boost::mutex send_mutex_shared_;
    boost::thread send_thread_;

    void sendImpl(const Image& image);

};

}

#endif
