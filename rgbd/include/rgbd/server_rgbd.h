#ifndef RGBD_SERVER_RGBD_H_
#define RGBD_SERVER_RGBD_H_

#include "rgbd/image.h"
#include "rgbd_msgs/GetRGBD.h"

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>

#include <mutex>
#include <thread>

namespace rgbd {

/**
 * @brief Server which provides RGBD topic and RGBD service
 */
class ServerRGBD {

public:

    /**
     * @brief Constructor
     */
    ServerRGBD(ros::NodeHandle nh=ros::NodeHandle());

    /**
     * @brief Destructor
     *
     * Nodehandle is shutdown and all threads are joined
     */
    virtual ~ServerRGBD();

    /**
     * @brief Initialize server
     * @param name Fully resolved server name
     * @param rgb_type rgb storage type
     * @param depth_type depth storage type
     * @param service_freq frequency of the thread processing service requests
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS, const float service_freq = 10);

    /**
     * @brief Write a new image to all interfaces
     * @param image Image to be written
     */
    void send(const Image& image);

    /**
     * @brief version of the RGBD message being used
     */
    const static int MESSAGE_VERSION;

protected:

    ros::NodeHandle nh_;
    ros::Publisher pub_image_;
    ros::ServiceServer service_server_;
    ros::CallbackQueue cb_queue_;

    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

    rgbd::Image image_;
    std::mutex image_mutex_;

    // Service thread
    std::thread service_thread_;

    /**
     * @brief serviceCallback
     * @param req Service Request
     * @param resp Service Response
     * @return success
     */
    bool serviceCallback(rgbd_msgs::GetRGBDRequest& req, rgbd_msgs::GetRGBDResponse& resp);

    /**
     * @brief Function to be called in the thread proving the service
     * @param frequency frequency for checking service requests
     */
    void serviceThreadFunc(const float frequency);

};

}

#endif // RGBD_SERVER_RGBD_H_
