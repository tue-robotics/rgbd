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
#include <ros/time.h>

#include <std_msgs/String.h>

#include <mutex>
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
     * @brief Initialize the client
     * @param server_name Fully resolved server name
     * @param timeout Timeout used to initialize each interface, currently only the ClientSHM interface requires a timeout
     * @return indicates success
     */
    bool initialize(const std::string& server_name, float timeout = 5.0);

    /**
     * @brief Calls deinitialize on implementaiton clients. Clears the #server_name_. #initialized will now return false.
     * @return indicates success
     */
    bool deinitialize();

    /**
     * @brief Check if the client is initialized. Checks if #server_name_ is set.
     * nextImage will not return an image if client is not initialized.
     * @return initialized or not
     */
    bool initialized() { return !server_name_.empty(); }

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

    enum class ClientImplMode {
        shm,
        rgbd
    };

    ClientRGBD client_rgbd_;

    ClientSHM client_shm_;

    ros::NodeHandle nh_;
    ros::CallbackQueue cb_queue_;
    ros::Subscriber sub_shm_hosts_;

    std::string hostname_;
    std::string server_name_;

    ros::WallTime last_time_shm_server_online_;

    std::thread sub_hosts_thread_;

    ClientImplMode client_impl_mode_;
    std::mutex switch_impl_mutex_;

    void hostsCallback(const std_msgs::StringConstPtr& msg);

    void subHostsThreadFunc(const float frequency);

};

}

#endif // RGBD_CLIENT_H_
