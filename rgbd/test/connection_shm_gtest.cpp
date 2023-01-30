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

class SHMInitialized : public SHM
{
protected:
    void SetUp() override
    {
        SHM::SetUp();
        server.send(rgbd::generateRandomImage());
        ros::Duration(0.01).sleep();
        EXPECT_TRUE(client.initialize(test_server_name));
        EXPECT_TRUE(client.initialized());
        EXPECT_TRUE(static_cast<boo>(client.nextImage()));
    }
};

TEST_F(SHM, InitializeBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(client.initialize(test_server_name, 1.));
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, InitializeAfterSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros:Duration(0.01).sleep();
    EXPECT_FALSE(client.initialized());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHM, DeInitialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros:Duration(0.01).sleep();
    EXPECT_FALSE(client.initialized());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_TRUE(client.deinitialize());
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImage)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImagePtr)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_TRUE(image2);
    if (image2) // This prevents a crash of the node. Test will still fail because of previous line
    {
        EXPECT_EQ(image, *image2);
    }
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImageTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
    image.setTimestamp(image.getTimestamp()+10.);
    server.send(image);
    ros::Duration(0.01).sleep();
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImageTwiceWithoutSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.nextImage(image2));
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImagePtrTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_TRUE(image2);
    if (image2) // This prevents a crash of the node. Test will still fail because of previous line
    {
        EXPECT_EQ(image, *image2);
    }
    EXPECT_FALSE(ros::isShuttingDown());
    image.setTimestamp(image.getTimestamp()+10.);
    image2.reset();
    server.send(image);
    ros::Duration(0.01).sleep();
    image2 = client.nextImage();
    EXPECT_TRUE(image2);
    if (image2) // This prevents a crash of the node. Test will still fail because of previous line
    {
        EXPECT_EQ(image, *image2);
    }
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(SHMInitialized, NextImagePtrTwiceWithoutSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_TRUE(image2);
    if (image2) // This prevents a crash of the node. Test will still fail because of previous line
    {
        EXPECT_EQ(image, *image2);
    }
    EXPECT_FALSE(ros::isShuttingDown());
    image2.reset();
    image2 = client.nextImage();
    EXPECT_FALSE(image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "shm_connection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
