#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/rate.h>

#include "rgbd/client.h"
#include "rgbd/view.h"

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please provide rgbd topic" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "rgbd_viewer");

    rgbd::Client client;
    client.intialize(ros::names::resolve(argv[1]));

    ros::Rate r(30);
    while (ros::ok())
    {
        if (!ros::master::check())
        {
            ROS_ERROR("Lost connection to master");
            return 1;
        }
        rgbd::Image image;
        if (!client.nextImage(image))
        {
            r.sleep();
            continue;
        }

        const cv::Mat& depth = image.getDepthImage();
        const cv::Mat& rgb = image.getRGBImage();

        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "time: " << ros::Time(image.getTimestamp()) << std::endl;

        if (depth.data)
        {
            rgbd::View view(image, depth.cols);
            const geo::DepthCamera& cam_model = view.getRasterizer();

            std::cout << "depth:" << std::endl;
            std::cout << "    camera model:" << std::endl;
            std::cout << "        fx, fy = " << cam_model.getFocalLengthX() << ", " << cam_model.getFocalLengthY() << std::endl;
            std::cout << "        cx, cy = " << cam_model.getOpticalCenterX() << ", " << cam_model.getOpticalCenterY() << std::endl;
            std::cout << "        Tx, Ty = " << cam_model.getOpticalTranslationX() << ", " << cam_model.getOpticalTranslationY() << std::endl;
            std::cout << "    size = " << depth.cols << " x " << depth.rows << std::endl;
        }
        else
        {
            std::cout << "depth: NO INFO" << std::endl;
        }

        if (rgb.data)
        {
            std::cout << "rgb:" << std::endl;
            std::cout << "    size = " << rgb.cols << " x " << rgb.rows << std::endl;
        }
        else
        {
            std::cout << "rgb: NO INFO" << std::endl;
        }

        r.sleep();
    }

    return 0;
}
