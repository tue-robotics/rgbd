#include "test_utils.h"

#include <gtest/gtest.h>

#include <rgbd/image.h>
#include <rgbd/server_rgbd.h>
#include <rgbd/client_rgbd.h>

#include <ros/init.h>
#include <ros/node_handle.h>


class RGBD : public testing::Test
{
protected:
    RGBD() : server(nh)
    {
    }

    virtual ~RGBD()
    {
        client.deinitialize();
        nh.shutdown();
    }

    void SetUp() override
    {
        image = rgbd::generateRandomImage();
        server.initialize(test_server_name);
    }

    const std::string test_server_name = "/test_ns/rgbd";

    ros::NodeHandle nh;
    rgbd::Image image;
    rgbd::ServerRGBD server;
    rgbd::ClientRGBD client;
};

TEST_F(RGBD, Initialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.initialized());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, DeInitialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.initialized());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    EXPECT_TRUE(client.deinitialize());
    EXPECT_FALSE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, NextImageBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    rgbd::Image image2;
    EXPECT_FALSE(client.nextImage(image2));
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, NextImagePtrBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_FALSE(image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, NextImage)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, NextImagePtr)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
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

TEST_F(RGBD, NextImageTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
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

TEST_F(RGBD, NextImageTwiceWithoutSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
    server.send(image);
    ros::Duration(0.01).sleep();
    rgbd::Image image2;
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.nextImage(image2));
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(RGBD, NextImagePtrTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
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

TEST_F(RGBD, NextImagePtrTwiceWithoutSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize(test_server_name));
    EXPECT_TRUE(client.initialized());
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
    ros::init(argc, argv, "rgbd_connection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
