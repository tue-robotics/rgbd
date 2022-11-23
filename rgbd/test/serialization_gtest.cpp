#include "test_utils.h"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <rgbd/image.h>
#include <rgbd/serialization.h>

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <sstream>


TEST(serialization, True)
{
    rgbd::Image image = rgbd::generateRandomImage();
    rgbd::Image image2;

    std::stringstream ss;
    tue::serialization::OutputArchive output_achive(ss);

    EXPECT_TRUE(rgbd::serialize(image, output_achive, rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS));
    tue::serialization::InputArchive input_achive(ss);
    EXPECT_TRUE(rgbd::deserialize(input_achive, image2));

    cv::Mat dst;
    const cv::Mat& image_depth = image.getDepthImage();
    const cv::Mat& image2_depth = image2.getDepthImage();
    cv::bitwise_xor(image_depth, image2_depth, dst);
    EXPECT_TRUE(cv::countNonZero(dst) == 0);

    const cv::Mat& image_rgb = image.getRGBImage();
    const cv::Mat& image2_rgb = image2.getRGBImage();
    cv::bitwise_xor(image_rgb, image2_rgb, dst);
    cv::Mat dst2;
    cv::transform(dst, dst2, cv::Matx<int, 1, 3>(1,1,1));
    EXPECT_TRUE(cv::countNonZero(dst2) == 0);

    EXPECT_TRUE(image.getCameraModel().cameraInfo() == image2.getCameraModel().cameraInfo());
    EXPECT_TRUE(image.getFrameId() == image2.getFrameId());
    EXPECT_TRUE(image.getTimestamp() == image2.getTimestamp());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
