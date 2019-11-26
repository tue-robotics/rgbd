#ifndef RGBD_TRANSPORT_CLIENTRAW_H_
#define RGBD_TRANSPORT_CLIENTRAW_H_

#include "Image.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>

namespace rgbd {

class ClientRaw {

public:

    ClientRaw();

    virtual ~ClientRaw();

    void setDepthTopic(const std::string& topic);

    void setCameraInfoTopic(const std::string& topic);

    bool nextImage(Image& image);

protected:

    ros::Subscriber sub_image_;
    ros::Subscriber sub_cam_info_;
    ros::CallbackQueue cb_queue_;

    bool received_image_;
    Image* image_ptr_;

    image_geometry::PinholeCameraModel cam_model_;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void depthImageCallback(sensor_msgs::ImageConstPtr depth_img_msg);

};

}

#endif
