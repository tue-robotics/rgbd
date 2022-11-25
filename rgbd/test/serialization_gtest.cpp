#include "test_utils.h"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rgbd/image.h>
#include <rgbd/serialization.h>

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <sstream>


class Serialization : public testing::Test
{
protected:
    void SetUp() override
    {
        image1 = rgbd::generateRandomImage();
    }

    static void compareImages(const rgbd::Image& image1, const rgbd::Image& image2)
    {
        const cv::Mat& image1_depth = image1.getDepthImage();
        const cv::Mat& image2_depth = image2.getDepthImage();
        cv::Mat dst;
        cv::bitwise_xor(image1_depth, image2_depth, dst);
        EXPECT_TRUE(cv::countNonZero(dst) == 0);

        const cv::Mat& image1_rgb = image1.getRGBImage();
        const cv::Mat& image2_rgb = image2.getRGBImage();
        cv::bitwise_xor(image1_rgb, image2_rgb, dst);
        cv::Mat dst2;
        cv::transform(dst, dst2, cv::Matx<int, 1, 3>(1,1,1));
        EXPECT_TRUE(cv::countNonZero(dst2) == 0);

        EXPECT_TRUE(image1.getCameraModel().cameraInfo() == image2.getCameraModel().cameraInfo());
        EXPECT_TRUE(image1.getFrameId() == image2.getFrameId());
        EXPECT_TRUE(image1.getTimestamp() == image2.getTimestamp());
    }

    rgbd::Image image1;
    rgbd::Image image2;
};

TEST_F(Serialization, LossLess)
{
    std::stringstream ss;
    tue::serialization::OutputArchive output_achive(ss);

    EXPECT_TRUE(rgbd::serialize(image1, output_achive, rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS));
    tue::serialization::InputArchive input_achive(ss);
    EXPECT_TRUE(rgbd::deserialize(input_achive, image2));

    compareImages(image1, image2);
}

TEST_F(Serialization, Lossy)
{
    std::stringstream ss;
    tue::serialization::OutputArchive output_achive(ss);

    EXPECT_TRUE(rgbd::serialize(image1, output_achive, rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_LOSSLESS));
    tue::serialization::InputArchive input_achive(ss);
    EXPECT_TRUE(rgbd::deserialize(input_achive, image2));

    std::vector<int> rgb_params;
    rgb_params.resize(2, 0);
    rgb_params[0] = cv::IMWRITE_JPEG_QUALITY;
    rgb_params[1] = 95; // default is 95

    std::vector<unsigned char> rgb_data;

    // Compress image
    cv::imencode(".jpg", image1.getRGBImage(), rgb_data, rgb_params);
    image1.setRGBImage(cv::imdecode(rgb_data, cv::IMREAD_UNCHANGED));

    compareImages(image1, image2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
