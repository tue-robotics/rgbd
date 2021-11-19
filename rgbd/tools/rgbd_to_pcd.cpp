#include <rgbd/serialization.h>
#include <rgbd/image.h>
#include <rgbd/view.h>

#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>

#include <opencv2/highgui/highgui.hpp>

#include <typeinfo>


int main(int argc, char **argv) {

    if (argc < 2)
    {
        std::cout << "Usage:\n\n   rgbd_to_pcd FILENAME.rgbd [FILENAME2.rgbd ...]\n\n";
        return 1;
    }

    for (uint i=1; i < argc; ++i)
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

        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        // Fill in the cloud data
        cloud.width = image.getRGBImage().cols;
        cloud.height = image.getRGBImage().rows;
        cloud.is_dense = false;
        cloud.resize (cloud.width * cloud.height);

        double fx = image.getCameraModel().fx();
        double fy = image.getCameraModel().fy();

        double half_height = 0.5 * cloud.height;
        double half_width = 0.5 * cloud.width;
        for (uint i=0; i < cloud.height; ++i)
        {
            for (uint j=0; j < cloud.width; ++j)
            {
                cv::Vec3b bgr = image.getRGBImage().at<cv::Vec3b>(i,j);
                double d = image.getDepthImage().at<float>(i,j);
                
                cloud.at(j,i).x = (-half_width+j) * d / fx;
                cloud.at(j,i).y = (-half_height+i) * d / fy;
                cloud.at(j,i).z = d;
                cloud.at(j,i).r = bgr[2];
                cloud.at(j,i).g = bgr[1];
                cloud.at(j,i).b = bgr[0];
            }
        }

        size_t lastindex = name.find_last_of(".");
        name = name.substr(0, lastindex);

        std::string pcd_filename = name + ".pcd";

        pcl::io::savePCDFileASCII (pcd_filename, cloud);
        std::cout << "Saved " << cloud.size () << " data points to " << pcd_filename << std::endl;
    }

    return 0;
}
