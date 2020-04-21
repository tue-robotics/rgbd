#ifndef RGBD_SERVER_RGBD_H_
#define RGBD_SERVER_RGBD_H_

#include "rgbd/image.h"
#include "rgbd_msgs/GetRGBD.h"

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>

#include <boost/thread.hpp>

namespace rgbd {

/**
 * @brief Server which provides RGBD topic and RGBD service
 */
class ServerRGBD {

public:

    /**
     * @brief Constructor
     */
    ServerRGBD();

    /**
     * @brief Destructor
     *
     * Nodehandle is shutdown and all threads are joined
     */
    virtual ~ServerRGBD();

    /**
     * @brief initialize initialize server
     * @param name Fully resolved server name
     * @param rgb_type rgb storage type
     * @param depth_type depth storage type
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS, const float service_freq = 10);

    /**
     * @brief send Write a new image to all interfaces
     * @param image Image to be written
     * @param threaded Use any threaded version of send of the interfaces
     */
    void send(const Image& image, bool threaded = false);

    /**
     * @brief MESSAGE_VERSION version of the RGBD message being used
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
    boost::mutex image_mutex_;

    // Service thread
    boost::thread service_thread_;

    // Send thread
    boost::thread send_thread_;

    /**
     * @brief sendImpl Implementaiton of send
     * @param image Image to be written
     */
    void sendImpl(const Image& image);

    /**
     * @brief serviceCallback
     * @param req Service Request
     * @param resp Service Response
     * @return succes
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
