#include <ros/ros.h>

#include "rgbd/ServerShmOnly.h"

#include "rgbd/ros/conversions.h"

rgbd::ServerShmOnly rgbd_server;

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_server_shm_only");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double max_fps = 30;
    nh_private.getParam("max_fps", max_fps);

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

    rgbd_server.initialize(ros::names::resolve("rgbd"), rgb_type, depth_type);

    ros::Subscriber sub_rgbd = nh.subscribe("rgbd", 1, &rgbd::ServerShmOnly::rgbdImageCallback, &rgbd_server);

    ros::Rate r(max_fps);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
