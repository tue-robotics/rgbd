#include <rgbd/serialization.h>
#include <rgbd/RGBDImage.h>

#include <fstream>

int main(int argc, char **argv) {

    std::string test_filename = "/tmp/rgbd_test_serialization";

    {
        rgbd::RGBDImage image;

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
    std::cout << "Image size: " << image.getWidth() << " x " << image.getHeight() << std::endl;

    return 0;
}
