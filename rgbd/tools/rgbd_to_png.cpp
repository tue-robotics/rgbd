#include <cstddef>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <ostream>
#include <rgbd/image.h>
#include <rgbd/serialization.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <string>

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        std::cout << "Usage:\n\n   rgbd_to_rgb_png FILENAME\n\n";
        return 1;
    }

    for (int i = 1; i < argc; ++i)
    {
        std::string name = std::string(argv[i]);

        // read
        std::ifstream f_in;
        f_in.open(name.c_str(), std::ifstream::binary);

        if (!f_in.is_open())
        {
            std::cerr << "Could not open '" << name << "'." << '\n';
            continue;
        }

        tue::serialization::InputArchive a_in(f_in);

        rgbd::Image image;
        rgbd::deserialize(a_in, image);

        const size_t lastindex = name.find_last_of('.');
        name = name.substr(0, lastindex);

        // write rgb image
        const std::string rgb_filename = name + "_rgb.png";

        if (cv::imwrite(rgb_filename, image.getRGBImage()))
            std::cout << "Succesfully stored '" << rgb_filename << "'" << '\n';
        else
            std::cerr << "Failed to write rgbd to rgb png" << '\n';

        // write depth image
        const std::string depth_filename = name + "_depth.png";

        if (cv::imwrite(depth_filename, image.getDepthImage()))
            std::cout << "Succesfully stored '" << depth_filename << "'" << '\n';
        else
            std::cerr << "Failed to write rgbd to depth png" << '\n';
    }

    return 0;
}
