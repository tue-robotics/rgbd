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
#include <iostream>
#include <sys/stat.h>
#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime

int main(int argc, char **argv)
{
    char key_pressed;
    ros::init(argc, argv, "rgbd_saver");

    ros::NodeHandle nh_private("~");

    float rate = 30;
    nh_private.getParam("rate", rate);

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
                ROS_FATAL("Lost connection to master");
                return 1;
            }
        }
        ROS_INFO("Press s to save and q to exit.");
        
        #pragma GCC diagnostic ignored "-Wunused-result"
        system ("/bin/stty raw");
        key_pressed = getchar();
        #pragma GCC diagnostic ignored "-Wunused-result"
        system ("/bin/stty cooked");
        
        if (key_pressed == 's')
        {
            if (client.nextImage(image))
            {
                std::stringstream ss;
                ss << "image_";
                const std::time_t time = image.getTimestamp();
                ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H.%M.%S.%f");
                ss << ".rgbd";
                const std::string file_name = ss.str();

                std::ofstream f_out;
                f_out.open(file_name.c_str(), std::ifstream::binary);
                try
                {
                    tue::serialization::OutputArchive a_out(f_out);
                    rgbd::serialize(image, a_out);
                    ROS_INFO_STREAM("Written image to '" << file_name << "'");
                }
                catch (const std::exception& e) // caught by reference to base
                {
                    ROS_ERROR_STREAM("Error while writing to '" << file_name << "':\n" << e.what());
                }
                f_out.close();
            }
        }
        else if (key_pressed == 'q')
        {
            ROS_INFO("Exiting");
            return 0;
        }
        r.sleep();
    }

    ROS_INFO("No image stored.");

    return 0;
}
