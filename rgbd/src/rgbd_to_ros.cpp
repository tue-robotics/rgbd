#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include "rgbd/client.h"
#include "rgbd/image.h"
#include "rgbd/server_ros.h"

#include <string>

int main(int argc, char **argv)
{
    std::vector<std::string> myargv;
    ros::removeROSArgs(argc, argv, myargv);
    bool publish_rgb = false, publish_depth = false, publish_pc = false;
    {
        bool valid_arg_provided = false;
        for (uint i = 1; i < myargv.size(); ++i)
        {
            const std::string& opt = myargv[i];
            if (opt == "-h" || opt == "--help")
            {
                std::cout << "Usage: rgbd_to_ros [OPTIONS]" << std::endl
                          << "    If no valid options are provided, rgb and depth images and camera info will be published" << std::endl
                          << "Options:" << std::endl
                          << "    -h, --help:         show this message" << std::endl
                          << "    -a, --all:          publish rgb, depth and pointcloud" << std::endl
                          << "    --rgb, --color:     publish rgb image and camera info" << std::endl
                          << "    --depth:            publish depth image and camera info" << std::endl
                          << "    --rgbd:             publish rgb and depth images and camera info" << std::endl
                          << "    --pc, --pointcloud: publish pointcloud" << std::endl;
                return 0;
            }
            else if (opt == "-a" || opt == "--all")
            {
                publish_rgb = true;
                publish_depth = true;
                publish_pc = true;
                valid_arg_provided = true;
            }
            else if (opt == "--rgb" || opt == "--color")
            {
                publish_rgb = true;
                valid_arg_provided = true;
            }
            else if (opt == "--depth")
            {
                publish_depth = true;
                valid_arg_provided = true;
            }
            else if (opt == "--rgbd")
            {
                publish_rgb = true;
                publish_depth = true;
                valid_arg_provided = true;
            }
            else if (opt == "--pc" || opt == "--pointcloud")
            {
                publish_pc = true;
                valid_arg_provided = true;
            }
            else if (!opt.compare(0, 2, "--"))
            {
                std::cout << "[rgbd_to_ros] Unknown option: '" << opt << "'." << std::endl;
                return 1;
            }
            else
            {
                std::cout << "[rgbd_to_ros] Ignoring option: '" << opt << "'." << std::endl;
            }
        }
        if (!valid_arg_provided)
        {
            publish_rgb = true;
            publish_depth = true;
        }
    }

    ros::init(argc, argv, "rgbd_to_ros");
    ros::start(); // Required to use ros::names::resolve, without creating a nodehandle
    
    ROS_DEBUG_STREAM("publish_rgb: " << publish_rgb);
    ROS_DEBUG_STREAM("publish_depth: " << publish_depth);
    ROS_DEBUG_STREAM("publish_pc: " << publish_pc);

    // Listener
    rgbd::Client client;
    client.initialize(ros::names::resolve("rgbd"));

    // Publishers
    rgbd::ServerROS server;
    server.initialize("", publish_rgb, publish_depth, publish_pc);

    ros::NodeHandle nh_private("~");
    float rate = 30;
    nh_private.getParam("rate", rate);

    rgbd::Image image;

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
        if (client.nextImage(image))
        {
            server.send(image);
        }

        r.sleep();
    }

    return 0;
}
