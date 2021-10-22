#include <rgbd/serialization.h>
#include <rgbd/image.h>
#include <rgbd/view.h>

#include <fstream>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

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
            std::cerr << "Could not open '" << name << "'." << std::endl;
            continue;
        }

        tue::serialization::InputArchive a_in(f_in);

        rgbd::Image image;
        rgbd::deserialize(a_in, image);

        /*
        size_t lastindex = name.find_last_of(".");
        name = name.substr(0, lastindex);

        std::string rgb_filename = name + "_rgb.png";

        if (cv::imwrite(rgb_filename, image.getRGBImage()))
            std::cout << "Succesfully stored '" << rgb_filename << "'" << std::endl;
        else
            std::cerr << "Failed to write rgbd to png" << std::endl;
        */
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Fill in the cloud data
        cloud.width    = image.getRGBImage().cols;
        cloud.height   = image.getRGBImage().rows;
        cloud.is_dense = false;
        cloud.resize (cloud.width * cloud.height);

        double fx = image.getCameraModel().fx();
        double fy = image.getCameraModel().fy();

        double half_height = 0.5 * cloud.height;
        double half_width = 0.5 * cloud.width;

        for (int i = 0; i < cloud.height; i++)
        {
            for (int j = 0; j < cloud.width; j++)
            {
                double d = image.getDepthImage().at<double>(i,j);
                double z = (half_height-i) * d / fy;
                double y = (-half_width+j) * d / fx;
            }
        }

        /*
        for (auto& point: cloud)
        {
            point.x = 1.0;
            point.y = 1.0;
            point.z = 1.0;
        }
        */

        pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
        std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
    }

    return 0;
}
