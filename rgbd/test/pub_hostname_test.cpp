#include <ros/init.h>
#include <ros/node_handle.h>

#include <rgbd/server_shm.h>

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_hostname_test");

    ros::NodeHandle nh;
    std::string hostname, rgbd_topic;
    if (!nh.getParam("hostname", hostname))
    {
        ROS_ERROR("Could not get param 'hostname'");
        return 1;
    }
    if (!nh.getParam("rgbd_topic", rgbd_topic))
    {
        ROS_ERROR("Could not get param 'rgbd_topic'");
        return 1;
    }
    std::thread thread = std::thread(rgbd::pubHostnameThreadFunc, std::ref(nh), rgbd_topic, hostname, 10);
    ros::Duration(5.).sleep();
    nh.shutdown();
    thread.join();
}
