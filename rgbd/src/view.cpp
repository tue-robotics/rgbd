#include <rgbd/serialization.h>
#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

    if (argc != 2)
    {
        std::cout << "Usage:\n\n   view FILENAME\n\n";
        return 1;
    }

    // read
    std::ifstream f_in;
    f_in.open(argv[1], std::ifstream::binary);

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << argv[1] << "'." << std::endl;
        return 1;
    }

    tue::serialization::InputArchive a_in(f_in);

    rgbd::Image image;
    rgbd::deserialize(a_in, image);

    std::cout << "Image loaded from disk." << std::endl;
    std::cout << "    name:  " << argv[1] << std::endl;
//    std::cout << "    size:  " << image.getWidth() << " x " << image.getHeight() << std::endl;
    std::cout << "    frame: " << image.getFrameId() << std::endl;
    std::cout << "    time:  " << ros::Time(image.getTimestamp()) << std::endl;

    cv::imshow("rgb", image.getRGBImage());
    cv::imshow("depth", image.getDepthImage() / 8);
    cv::waitKey();

    return 0;
}
