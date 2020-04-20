/**
 * This client listens to RGBD messages on a single topic or via shared memory, directy rgbd::Image.
 */

#ifndef RGBD_CLIENT_H_
#define RGBD_CLIENT_H_

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include "rgbd/types.h"
#include "rgbd/Client_RGBD.h"
#include "rgbd/Client_SHM.h"

#include "rgbd_msgs/RGBD.h"


namespace rgbd {

class Client {

public:

    Client();

    virtual ~Client();

    /**
     * @brief intialize Initialize the client
     * @param server_name Fully resolved server name
     */
    void intialize(const std::string& server_name, float timeout = 5.0);

    bool initialized() { return (client_shm_.initialized() || client_rgbd_.initialized()); }

    bool nextImage(Image& image);

    ImagePtr nextImage();

protected:

    ClientRGBD client_rgbd_;

    ClientSHM client_shm_;

};

}

#endif // RGBD_CLIENT_H_
