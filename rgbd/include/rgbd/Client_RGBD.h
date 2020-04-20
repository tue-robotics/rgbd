/**
 * This client listens to RGBD messages on a single topic directing rgbd::Image.
 */

#ifndef RGBD_CLIENT_RGBD_H_
#define RGBD_CLIENT_RGBD_H_

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include "rgbd/types.h"
#include "rgbd/shared_mem_client.h"

#include "rgbd_msgs/RGBD.h"


namespace rgbd {

class ClientRGBD {

public:

    ClientRGBD();

    virtual ~ClientRGBD();

    /**
     * @brief intialize Initialize the client
     * @param server_name Fully resolved server name
     */
    void intialize(const std::string& server_name);

    bool initialized() { return !sub_image_.getTopic().empty(); }

    bool nextImage(Image& image);

    ImagePtr nextImage();

protected:

    ros::NodeHandle nh_;

    ros::Subscriber sub_image_;
    ros::CallbackQueue cb_queue_;

    bool new_image_;
    Image* image_ptr_;

    void rgbdImageCallback(const rgbd_msgs::RGBD::ConstPtr& msg);

};

}

#endif // RGBD_CLIENT_RGBD_H_
