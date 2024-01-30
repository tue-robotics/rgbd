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
        
        //std::cin >> key_pressed;
        system ("/bin/stty raw");
        key_pressed = getchar();
        
        if (key_pressed == 's')
        {
            system ("/bin/stty cooked");
            if (client.nextImage(image))
            {
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
                auto file_name = ss.str();
                const char* file_name_new = file_name.c_str(); 
                // write
                std::ofstream f_out;
                f_out.open(file_name_new, std::ifstream::binary);
                tue::serialization::OutputArchive a_out(f_out);
                rgbd::serialize(image, a_out);
                f_out.close();
                ROS_INFO("Image stored to disk.");
            }
        }
        if (key_pressed == 'q')
        {
            system ("/bin/stty cooked");
            return 0;
        }
        r.sleep();
    }

    ROS_INFO("No image stored.");

    return 0;
}
