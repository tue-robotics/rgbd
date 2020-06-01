#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

#include "rgbd/client_rgbd.h"
#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/tools.h"
#include "rgbd/types.h"

#include <std_msgs/String.h>

#include <thread>

// ----------------------------------------------------------------------------------------

void pubHostnameThreadFunc(ros::NodeHandle& nh, const std::string server_name, const std::string hostname, const float frequency)
{
    ros::Publisher pub_shm_hostname = nh.advertise<std_msgs::String>(server_name + "/hosts", 1);
    ros::Rate r(frequency);
    std_msgs::String msg;
    msg.data = hostname;
    while(nh.ok())
    {
        pub_shm_hostname.publish(msg);
        r.sleep();
    }
}

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

    client.intialize(server_name);
    server.initialize(server_name);

    ros::NodeHandle nh;
    std::thread pub_hostname_thread(pubHostnameThreadFunc, std::ref(nh), server_name, host_name, 10);

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
            server.send(*image_ptr);
        r.sleep();
    }

    nh.shutdown();
    pub_hostname_thread.join();

    return 0;
}
