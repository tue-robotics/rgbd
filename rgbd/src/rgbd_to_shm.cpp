#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>

#include "rgbd/client_rgbd.h"
#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/tools.h"
#include "rgbd/types.h"

#include <std_msgs/String.h>

#include <functional>
#include <memory>
#include <thread>


// ----------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_to_shm");

    ros::NodeHandle nh_private("~");

    double rate = 30;
    nh_private.getParam("rate", rate);

    // ------------------------------

    rgbd::ClientRGBD client;
    rgbd::ServerSHM server;

    const std::string server_name = ros::names::resolve("rgbd");
    const std::string host_name = rgbd::get_hostname();

    client.initialize(server_name);
    server.initialize(server_name);

    ros::NodeHandle nh;
    std::unique_ptr<std::thread> pub_hostname_thread_ptr(nullptr);

    rgbd::ImagePtr image_ptr;

    ros::Rate r(rate);
    while (ros::ok())
    {
        if (!ros::master::check())
        {
            ROS_ERROR("Lost connection to master");
            return 1;
        }
        image_ptr = client.nextImage();
        if (image_ptr)
        {
            if (!pub_hostname_thread_ptr)
                pub_hostname_thread_ptr = std::unique_ptr<std::thread>(new std::thread(rgbd::pubHostnameThreadFunc, std::ref(nh), server_name, host_name, 10));
            server.send(*image_ptr);
        }
        r.sleep();
    }

    nh.shutdown();
    if(pub_hostname_thread_ptr)
        pub_hostname_thread_ptr->join();

    return 0;
}
