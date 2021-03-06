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

#include <rgbd/serialization.h>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_saver");

    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

    std::string file_name = "rgbd_image";

    rgbd::Client client;
    client.initialize(ros::names::resolve("rgbd"));

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
                ROS_ERROR("Lost connection to master");
                return 1;
            }
        }
        if (client.nextImage(image))
        {
            // write
            std::ofstream f_out;
            f_out.open(file_name.c_str(), std::ifstream::binary);
            tue::serialization::OutputArchive a_out(f_out);
            rgbd::serialize(image, a_out);
            f_out.close();

            ROS_INFO("Image stored to disk.");
            return 0;
        }

        r.sleep();
    }

    ROS_INFO("No image stored.");

    return 0;
}
