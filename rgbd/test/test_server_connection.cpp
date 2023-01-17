#include "test_utils.h"

#include <rgbd/image.h>
#include <rgbd/server_rgbd.h>

#include <ros/master.h>
#include <ros/names.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_server");
    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    rgbd::ServerRGBD server;
    server.initialize(ros::names::resolve("rgbd"));

    ros::Rate r(rate);

    ros::WallTime last_master_check = ros::WallTime::now();

    rgbd::Image image;
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
        image = rgbd::generateRandomImage();
        image.setFrameId("test_frame");
        image.setTimestamp(ros::Time::now().toSec());

        server.send(image);

        r.sleep();
    }

    return 0;
}
