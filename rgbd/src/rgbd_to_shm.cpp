#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>

#include "rgbd/client_rgbd.h"
#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/types.h"

// ----------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_to_shm");

    ros::NodeHandle nh_private("~");

    double max_fps = 30;
    nh_private.getParam("max_fps", max_fps);

    // ------------------------------

    rgbd::ClientRGBD client;
    rgbd::ServerSHM server;

    const std::string server_name = ros::names::resolve("rgbd");

    client.intialize(server_name);
    server.initialize(server_name);

    rgbd::ImagePtr image_ptr;

    ros::Rate r(max_fps);
    while (ros::ok())
    {
        if (!ros::master::check())
        {
            ROS_ERROR("Lost connection to master");
            return 1;
        }
        image_ptr = client.nextImage();
        if (image_ptr)
            server.send(*image_ptr);
        r.sleep();
    }

    return 0;
}
