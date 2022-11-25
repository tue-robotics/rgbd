#include "test_utils.h"

#include <gtest/gtest.h>

#include <boost/interprocess/shared_memory_object.hpp>

#include <rgbd/image.h>
#include <rgbd/server_shm.h>
#include <rgbd/utility.h>

#include <memory>
#include <sstream>
#include <thread>


namespace ipc = boost::interprocess;

class SHM : public testing::Test
{
protected:
    void SetUp() override
    {
        image = rgbd::generateRandomImage();
        server.initialize(test_server_name);
    }

    const std::string test_server_name = "/test_ns/test_name";
    const std::string test_server_name_shm = "-test_ns-test_name";

    rgbd::Image image;
    rgbd::ServerSHM server;
};

TEST_F(SHM, DeleteSHM)
{
    EXPECT_FALSE(ros::isShuttingDown());
    ros::NodeHandle nh;
    server.send(image);
    EXPECT_FALSE(ros::isShuttingDown());
    ipc::shared_memory_object::remove(test_server_name_shm.c_str());
    ros::Duration(1.1).sleep(); // Give thread some time to run
    EXPECT_TRUE(ros::isShuttingDown());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "shm_behaviour");

    return RUN_ALL_TESTS();
}
