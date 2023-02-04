#include "test_utils.h"

#include <gtest/gtest.h>

#include <opencv2/core/mat.hpp>

#include <rgbd/image.h>

#include <sensor_msgs/CameraInfo.h>


TEST(Image, EmptyClone)
{
    rgbd::Image image1;
    rgbd::Image image2 = image1.clone();

    EXPECT_EQ(image1, image2);
    EXPECT_FALSE(image1 != image2);
}

TEST(Image, RandomClone)
{
    rgbd::Image image1 = rgbd::generateRandomImage();
    rgbd::Image image2 = image1.clone();

    EXPECT_EQ(image1, image2);
    EXPECT_FALSE(image1 != image2);
}

TEST(Image, NotEqual)
{
    rgbd::Image image1 = rgbd::generateRandomImage();
    rgbd::Image image2 = rgbd::generateRandomImage();

    // Two random images
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);

    // Different frame_id
    image2 = image1.clone();
    image2.setFrameId("bla");
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);

    // Different timestamp
    image2 = image1.clone();
    image2.setTimestamp(image1.getTimestamp()+10.);
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);

    // Different camera model
    image2 = image1.clone();
    sensor_msgs::CameraInfo cam_info = image2.getCameraModel().cameraInfo();
    cam_info.width += 10;
    image2.setCameraInfo(cam_info);
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);

    // Different depth image
    image2 = image1.clone();
    cv::Mat depth = image2.getDepthImage();
    depth.at<float>(0, 0) += 0.1;
    image2.setDepthImage(depth);
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);

    // Different RGB color image
    image2 = image1.clone();
    cv::Mat color = image2.getRGBImage();
    color.at<cv::Vec3b>(0, 0) *= 2;
    image2.setRGBImage(color);
    EXPECT_FALSE(image1 == image2);
    EXPECT_NE(image1, image2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
