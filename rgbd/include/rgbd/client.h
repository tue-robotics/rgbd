/**
 * This client listens to RGBD messages on a single topic or via shared memory, directy rgbd::Image.
 */

#ifndef RGBD_CLIENT_H_
#define RGBD_CLIENT_H_

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include "rgbd/types.h"
#include "rgbd/client_rgbd.h"
#include "rgbd/client_shm.h"

#include "rgbd_msgs/RGBD.h"


namespace rgbd {

/**
 * @brief Client which provides the interfaces of ClientRGBD and ClientSHM
 */
class Client {

public:

    /**
     * @brief Constructor
     */
    Client();

    /**
     * @brief Destructor
     */
    virtual ~Client();

    /**
     * @brief intialize Initialize the client
     * @param server_name Fully resolved server name
     */
    void intialize(const std::string& server_name, float timeout = 5.0);

    /**
     * @brief Check if the client is initialized. First checks if ClientSHM is initialized, otherwise ClientRGBD.
     * nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return (client_shm_.initialized() || client_rgbd_.initialized()); }

    /**
     * @brief Get a new Image. If no new image has been received since the last call,
     * no image will be written and false will be returned.
     * @param image Image reference which will be written.
     * @return valid image written
     */
    bool nextImage(Image& image);

    /**
     * @brief Get a new Image. If no new image has been received since the last call,
     * The ImagePtr will be a nullptr
     * @return ImagePtr to an Image or a nullptr
     */
    ImagePtr nextImage();

protected:

    ClientRGBD client_rgbd_;

    ClientSHM client_shm_;

};

}

#endif // RGBD_CLIENT_H_
