#include "test_utils.h"

#include <gtest/gtest.h>

#include <rgbd/image.h>
#include <rgbd/server_shm.h>
#include <rgbd/client_shm.h>

#include <ros/init.h>


class SHM : public testing::Test
{
protected:
    void SetUp() override
    {
        image = rgbd::generateRandomImage();
        server.initialize(test_server_name);
    }

    const std::string test_server_name = "/test_ns/rgbd";

    rgbd::Image image;
    rgbd::ServerSHM server;
    rgbd::ClientSHM client;
};

TEST_F(SHM, InitializeBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.initialize(test_server_name, 1.));
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, InitializeAfterSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, DeInitialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_TRUE(client.deinitialize());
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, NextImage)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_TRUE(client.initialize(test_server_name));
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, NextImagePtr)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_TRUE(client.initialize(test_server_name));
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_TRUE(image2);
    EXPECT_EQ(image, *image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, NextImageTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    EXPECT_TRUE(client.initialize(test_server_name));
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
    image.setTimestamp(1000.);
    server.send(image);
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "shm_connection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
