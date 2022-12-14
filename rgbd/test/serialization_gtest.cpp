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

    EXPECT_EQ(image1, image2);
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

    EXPECT_EQ(image1, image2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
