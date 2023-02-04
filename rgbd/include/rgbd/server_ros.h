#ifndef RGBD_SERVER_ROS_H_
#define RGBD_SERVER_ROS_H_

#include "rgbd/types.h"

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <memory>

namespace rgbd {

/**
 * @brief Server which publishes ROS rgb image, depth image and pointcloud messages
 */
class ServerROS {

public:

    /**
     * @brief Constructor
     */
    ServerROS(ros::NodeHandle nh=ros::NodeHandle());

    /**
     * @brief Destructor
     */
    virtual ~ServerROS();

    /**
     * @brief initialize server
     * @param ns relative or absolute namespace of publishers
     * @param publish_rgb Publish rgb image and camera info
     * @param publish_depth Publish depth image and camera info
     * @param publish_pc Publish point cloud
     */
    void initialize(std::string ns = "", const bool publish_rgb = false, const bool publish_depth = false, const bool publish_pc = false);

    /**
     * @brief Publish a new image to the selected ROS topics
     * @param image Image to be published
     */
    void send(const Image& image);

protected:

    ros::NodeHandle nh_;
    std::shared_ptr<ros::Publisher> pub_rgb_img_;
    std::shared_ptr<ros::Publisher> pub_rgb_info_;
    std::shared_ptr<ros::Publisher> pub_depth_img_;
    std::shared_ptr<ros::Publisher> pub_depth_info_;
    std::shared_ptr<ros::Publisher> pub_depth_pc_;

};

}

#endif // RGBD_SERVER_ROS_H_
