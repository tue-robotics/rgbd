#include "rgbd/Client.h"
#include "rgbd/Image.h"

#include <rgbd/serialization.h>
#include <fstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_saver");

    std::string file_name = "rgbd_image";

    rgbd::Client client;
    client.intialize("rgbd");

    ros::Rate r(30);
    while (ros::ok()) {
        rgbd::Image image;
        if (client.nextImage(image)) {

            // write
            std::ofstream f_out;
            f_out.open(file_name.c_str(), std::ifstream::binary);
            tue::serialization::OutputArchive a_out(f_out);

            rgbd::serialize(image, a_out);
            f_out.close();

            std::cout << "Image stored to disk." << std::endl;
            return 0;
        }

        r.sleep();
    }

    std::cout << "No image stored." << std::endl;

    return 0;
}
