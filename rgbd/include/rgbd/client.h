/**
 * This client listens to RGBD messages on a single topic or via shared memory, provides rgbd::Image.
 */

#ifndef RGBD_CLIENT_H_
#define RGBD_CLIENT_H_

#include "rgbd/types.h"
#include "rgbd/client_rgbd.h"
#include "rgbd/client_shm.h"

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <std_msgs/String.h>

#include <thread>


namespace rgbd {

/**
 * @brief Client which uses the interfaces of ClientRGBD and ClientSHM
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
     * @param timeout Timeout used to initialize each interface, currently only the ClientSHM interface requires a timeout
     * @return indicates success
     */
    bool intialize(const std::string& server_name, float timeout = 5.0);

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

    ros::NodeHandle nh_;
    ros::CallbackQueue cb_queue_;
    ros::Subscriber sub_shm_hosts_;

    std::string hostname_;

    std::thread sub_hosts_thread_;

    void hostsCallback(const std_msgs::StringConstPtr& msg);

    void subHostsThreadFunc(const float frequency);

};

}

#endif // RGBD_CLIENT_H_
