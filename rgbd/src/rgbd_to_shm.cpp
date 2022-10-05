#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

#include "rgbd/client_rgbd.h"
#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/types.h"
#include "rgbd/utility.h"

#include <std_msgs/String.h>

#include <functional>
#include <memory>
#include <thread>


class Node
{
public:
    Node() : rate_(30), server_name_(ros::names::resolve("rgbd")), host_name_(rgbd::get_hostname())
    {
        ros::NodeHandle nh_private("~");
        nh_private.getParam("rate", rate_);

        client_.initialize(server_name_);
        server_.initialize(server_name_);
    }

    virtual ~Node()
    {
        nh_.shutdown();
        if (pub_hostname_thread_ptr_)
            pub_hostname_thread_ptr_->join();
    }

    int run()
    {
        rgbd::ImagePtr image_ptr;

        ros::WallTime last_master_check = ros::WallTime::now();

        ros::Rate r(rate_);
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
            image_ptr = client_.nextImage();
            if (image_ptr)
            {
                if (!pub_hostname_thread_ptr_)
                    pub_hostname_thread_ptr_ = std::make_unique<std::thread>(rgbd::pubHostnameThreadFunc, std::ref(nh_), server_name_, host_name_, 10);
                server_.send(*image_ptr);
            }
            r.sleep();
        }

        return 0;
    }

private:
    ros::NodeHandle nh_;

    rgbd::ClientRGBD client_;
    rgbd::ServerSHM server_;

    double rate_;

    const std::string server_name_;
    const std::string host_name_;

    std::unique_ptr<std::thread> pub_hostname_thread_ptr_;
};


// ----------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_to_shm");

    Node node;

    return node.run();
}
