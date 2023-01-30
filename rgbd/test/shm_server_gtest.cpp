#include "test_utils.h"

#include <gtest/gtest.h>

#include <boost/interprocess/shared_memory_object.hpp>

#include <rgbd/image.h>
#include <rgbd/server_shm.h>

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <std_msgs/String.h>

#include <memory>
#include <thread>


namespace ipc = boost::interprocess;


class SHMServer : public testing::Test
{
protected:
    void SetUp() override
    {
        image = rgbd::generateRandomImage();
        server.initialize(test_server_name);
    }

    const std::string test_server_name = "/test_ns/rgbd";
    const std::string test_server_name_shm = "-test_ns-rgbd";

    rgbd::Image image;
    rgbd::ServerSHM server;
};

class SHMServerHostame : public SHMServer
{
protected:
    SHMServerHostame() : SHMServer(), correct_hostname(true)
    {
    }

    void SetUp() override
    {
        sub = nh.subscribe<std_msgs::String>(test_server_name_hosts, 10, &SHMServerHostame::hostsCallback, this);
    }

    void hostsCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (msg->data != test_hostname)
            correct_hostname = false;
    }

    const std::string test_server_name_hosts = "/test_ns/rgbd/hosts";
    const std::string test_hostname = "test_hostname";

    ros::NodeHandle nh;
    ros::Subscriber sub;
    bool correct_hostname;
};

TEST_F(SHMServer, Initialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_THROW(ipc::shared_memory_object(ipc::open_only, test_server_name_shm.c_str(), ipc::read_write), ipc::interprocess_exception);
}

TEST_F(SHMServer, SHMCreated)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_FALSE(ros::isShuttingDown());
    ros::Duration(1.1).sleep(); // Give thread some time to run
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMServer, DeleteSHM)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_FALSE(ros::isShuttingDown());
    ipc::shared_memory_object::remove(test_server_name_shm.c_str());
    ros::Duration(1.1).sleep(); // Give thread some time to run
    EXPECT_TRUE(ros::isShuttingDown());
}

TEST_F(SHMServerHostame, PubHostname)
{
    std::thread thread = std::thread(rgbd::pubHostnameThreadFunc, std::ref(nh), test_server_name, test_hostname, 10);
    ros::Duration(5.).sleep();
    nh.shutdown();
    thread.join();
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(correct_hostname);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "shm_server_behaviour");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
