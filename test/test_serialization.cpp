#include <rgbd/serialization.h>
#include <rgbd/RGBDImage.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

    std::string test_filename = "/tmp/rgbd_test_image";

    {
        image_geometry::PinholeCameraModel cam_model;

        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
        cv::Mat depth_image(480, 640, CV_32FC1, 3);

        rgbd::RGBDImage image(rgb_image, depth_image, cam_model, "no_frame", 0);

        // write
        std::ofstream f_out;
        f_out.open(test_filename.c_str(), std::ifstream::binary);

        tue::serialization::OutputArchive a_out(f_out, 0);

        rgbd::serialize(image, a_out);

        f_out.close();
    }

    std::cout << "Image stored to disk." << std::endl;

    rgbd::RGBDImage image;

    {
        // read
        std::ifstream f_in;
        f_in.open(test_filename.c_str(), std::ifstream::binary);

        tue::serialization::InputArchive a_in(f_in);

        rgbd::deserialize(a_in, image);
    }

    std::cout << "Image loaded from disk." << std::endl;
    std::cout << "    size:  " << image.getWidth() << " x " << image.getHeight() << std::endl;
    std::cout << "    frame: " << image.getFrameId() << std::endl;
    std::cout << "    time:  " << ros::Time(image.getTimestamp()) << std::endl;

    cv::imshow("rgb", image.getOriginalRGBImage());
    cv::imshow("depth", image.getOriginalDepthImage() / 8);
    cv::waitKey();

    return 0;
}
