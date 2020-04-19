#ifndef SERVER_RGBD_H
#define SERVER_RGBD_H

#include "rgbd/Image.h"
#include "rgbd_msgs/GetRGBD.h"

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>

#include <boost/thread.hpp>

namespace rgbd {

class ServerRGBD {

public:

    ServerRGBD();

    virtual ~ServerRGBD();

    /**
     * @brief initialize initialize server
     * @param name Fully resolved server name
     * @param rgb_type
     * @param depth_type
     */
    void initialize(const std::string& name, RGBStorageType rgb_type = RGB_STORAGE_LOSSLESS, DepthStorageType depth_type = DEPTH_STORAGE_LOSSLESS, const float service_freq = 10);

    void send(const Image& image, bool threaded = false);

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

    void sendImpl(const Image& image);

    bool serviceCallback(rgbd_msgs::GetRGBDRequest& req, rgbd_msgs::GetRGBDResponse& resp);

    void serviceThreadFunc(const float frequency);

};

}

#endif // SERVER_RGBD_H
