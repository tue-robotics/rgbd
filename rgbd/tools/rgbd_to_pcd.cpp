#include <cstddef>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <pcl/point_cloud.h>
#include <rgbd/image.h>
#include <rgbd/serialization.h>

#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>

#include <opencv2/highgui/highgui.hpp>

#include <string>

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        std::cout << "Usage:\n\n   rgbd_to_pcd FILENAME.rgbd [FILENAME2.rgbd ...]\n\n";
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

        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        // Fill in the cloud data
        cloud.width = image.getRGBImage().cols;
        cloud.height = image.getRGBImage().rows;
        cloud.is_dense = false;
        cloud.resize(static_cast<std::size_t>(cloud.width) * cloud.height);

        double const fx = image.getCameraModel().fx();
        double const fy = image.getCameraModel().fy();

        double const half_height = 0.5 * cloud.height;
        double const half_width = 0.5 * cloud.width;
        for (uint i = 0; i < cloud.height; ++i)
        {
            for (uint j = 0; j < cloud.width; ++j)
            {
                int const ii = static_cast<int>(i);
                int const jj = static_cast<int>(j);
                cv::Vec3b bgr = image.getRGBImage().at<cv::Vec3b>(ii, jj);
                double const d = image.getDepthImage().at<float>(ii, jj);

                // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access) pcl::PointXYZRGB exposes its fields via a
                // union; there is no non-union accessor.
                cloud.at(jj, ii).x = static_cast<float>((-half_width + j) * d / fx);
                cloud.at(jj, ii).y = static_cast<float>((-half_height + i) * d / fy);
                cloud.at(jj, ii).z = d;
                cloud.at(jj, ii).r = bgr[2];
                cloud.at(jj, ii).g = bgr[1];
                cloud.at(jj, ii).b = bgr[0];
                // NOLINTEND(cppcoreguidelines-pro-type-union-access)
            }
        }

        size_t const lastindex = name.find_last_of('.');
        name = name.substr(0, lastindex);

        std::string const pcd_filename = name + ".pcd";

        pcl::io::savePCDFileASCII(pcd_filename, cloud);
        std::cout << "Saved " << cloud.size() << " data points to " << pcd_filename << '\n';
    }

    return 0;
}
