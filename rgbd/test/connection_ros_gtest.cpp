#include "test_utils.h"

#include <gtest/gtest.h>

#include <rgbd/image.h>
#include <rgbd/server_ros.h>
#include <rgbd/client_ros.h>

#include <ros/init.h>
#include <ros/console.h>


class ROS : public testing::Test
{
protected:
    ROS() : server(nh)
    {
    }

    virtual ~ROS()
    {
        client.deinitialize();
        nh.shutdown();
    }

    void SetUp() override
    {
        image = rgbd::generateRandomImage();
        server.initialize("", true, true, false);
    }

    ros::NodeHandle nh;
    rgbd::Image image;
    rgbd::ServerROS server;
    rgbd::ClientROS client;
};

TEST_F(ROS, Initialize)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_FALSE(client.initialized());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(ROS, NextImageBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    rgbd::Image image2;
    EXPECT_FALSE(client.nextImage(image2));
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(ROS, NextImagePtrBeforeSend)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    rgbd::ImagePtr image2 = client.nextImage();
    EXPECT_FALSE(image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(ROS, NextImage)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    rgbd::Image image2;
    for (uint i=0; i<2; ++i)
    {
        client.nextImage(image2);
        server.send(image);
        ros::Duration(0.05).sleep();
    }
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(ROS, NextImagePtr)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    rgbd::ImagePtr image2;
    for (uint i=0; i<2; ++i)
    {
        image2 = client.nextImage();
        server.send(image);
        ros::Duration(0.05).sleep();
    }
    image2 = client.nextImage();
    EXPECT_TRUE(image2);
    if (image2) // This prevents a crash of the node. Test will still fail because of previous line
    {
        EXPECT_EQ(image, *image2);
    }
    EXPECT_FALSE(ros::isShuttingDown());
}

TEST_F(ROS, NextImageTwice)
{
    EXPECT_FALSE(ros::isShuttingDown());
    EXPECT_TRUE(client.initialize("rgb/image", "depth/image", "rgb/camera_info"));
    EXPECT_TRUE(client.initialized());
    rgbd::Image image2;
    for (uint i=0; i<2; ++i)
    {
        client.nextImage(image2);
        server.send(image);
        ros::Duration(0.05).sleep();
    }
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
    image.setTimestamp(image.getTimestamp()+10.);
    server.send(image);
    ros::Duration(0.05).sleep();
    EXPECT_TRUE(client.nextImage(image2));
    EXPECT_EQ(image, image2);
    EXPECT_FALSE(ros::isShuttingDown());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_connection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
