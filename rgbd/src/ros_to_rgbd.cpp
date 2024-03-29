#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include "rgbd/client_ros.h"
#include "rgbd/image.h"
#include "rgbd/server.h"
#include "rgbd/types.h"

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_to_rgbd");

    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    // ----- READ RGB STORAGE TYPE

    rgbd::RGBStorageType rgb_type;

    std::string rgb_type_str = "lossless";
    nh_private.getParam("rgb_storage", rgb_type_str);
    if (rgb_type_str == "none")
        rgb_type = rgbd::RGB_STORAGE_NONE;
    else if (rgb_type_str == "lossless")
        rgb_type = rgbd::RGB_STORAGE_LOSSLESS;
    else if (rgb_type_str == "jpg")
        rgb_type = rgbd::RGB_STORAGE_JPG;
    else
    {
        ROS_ERROR("Unknown 'rgb_storage' type: should be 'none', 'lossless', or 'jpg'.");
        return 1;
    }

    // ----- READ DEPTH STORAGE TYPE

    rgbd::DepthStorageType depth_type;

    std::string depth_type_str = "lossless";
    nh_private.getParam("depth_storage", depth_type_str);
    if (depth_type_str == "none")
        depth_type = rgbd::DEPTH_STORAGE_NONE;
    else if (depth_type_str == "lossless")
        depth_type = rgbd::DEPTH_STORAGE_LOSSLESS;
    else if (depth_type_str == "png")
        depth_type = rgbd::DEPTH_STORAGE_PNG;
    else
    {
        ROS_ERROR("Unknown 'depth_storage' type: should be 'none', 'lossless', or 'png'.");
        return 1;
    }

    // ------------------------------

    rgbd::ClientROS client;
    rgbd::Server server;

    client.initialize("rgb_image", "depth_image", "cam_info");
    server.initialize(ros::names::resolve("rgbd"), rgb_type, depth_type);

    rgbd::ImagePtr image_ptr;

    ros::WallTime last_master_check = ros::WallTime::now();

    ros::Rate r(rate);
    while (ros::ok())
    {
        if (ros::WallTime::now() >= last_master_check + ros::WallDuration(1))
        {
            last_master_check = ros::WallTime::now();
            if (!ros::master::check())
            {
                ROS_FATAL("Lost connection to master");
                return 1;
            }
        }
        image_ptr = client.nextImage();
        if (image_ptr)
            server.send(*image_ptr);
        r.sleep();
    }

    return 0;
}
