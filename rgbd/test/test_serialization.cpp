#include <rgbd/serialization.h>
#include <rgbd/View.h>
#include <rgbd/Image.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

    std::string test_filename = "/tmp/rgbd_test_image";

    {
        image_geometry::PinholeCameraModel cam_model;

        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
        cv::line(rgb_image, cv::Point2i(100, 100), cv::Point2i(200, 300), cv::Scalar(255, 0, 0), 3);
        cv::Mat depth_image(480, 640, CV_32FC1, 3);
        cv::line(depth_image, cv::Point2i(100, 100), cv::Point2i(200, 300), cv::Scalar(1), 3);

        rgbd::Image image(rgb_image, depth_image, cam_model, "no_frame", 0);

        // write
        std::ofstream f_out;
        f_out.open(test_filename.c_str(), std::ifstream::binary);

        tue::serialization::OutputArchive a_out(f_out);

        if (!rgbd::serialize(image, a_out, rgbd::RGB_STORAGE_LOSSLESS, rgbd::DEPTH_STORAGE_LOSSLESS))
        {
            std::cout << "Could not store image to disk" << std::endl;
            return 1;
        }
    }

    std::cout << "Image stored to disk." << std::endl;

    rgbd::Image image;

    {
        // read
        std::ifstream f_in;
        f_in.open(test_filename.c_str(), std::ifstream::binary);

        tue::serialization::InputArchive a_in(f_in);

        rgbd::deserialize(a_in, image);
    }

    std::cout << "Image loaded from disk." << std::endl;
//    std::cout << "    size:  " << image.getWidth() << " x " << image.getHeight() << std::endl;
    std::cout << "    frame: " << image.getFrameId() << std::endl;
    std::cout << "    time:  " << ros::Time(image.getTimestamp()) << std::endl;

    if (image.getRGBImage().data)
        cv::imshow("rgb", image.getRGBImage());
    else
        std::cout << "File did not contain RGB data." << std::endl;

    if (image.getDepthImage().data)
        cv::imshow("depth", image.getDepthImage() / 8);
    else
        std::cout << "File did not contain Depth data" << std::endl;
    cv::waitKey();

    return 0;
}
