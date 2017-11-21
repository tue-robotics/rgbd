#include "rgbd/Client.h"
#include "rgbd/View.h"
//#include <opencv2/highgui/highgui.hpp>

// Point cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please specify rgbd topic / name" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "rgbd_to_pc");

    // Listener
    rgbd::Client client;
    client.intialize(argv[1]);

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_depth_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("depth/points", 1);

    ros::Time last_image_stamp;

    ros::Rate r(30);
    while (ros::ok())
    {
        if (!last_image_stamp.isZero() && ros::Time::now() - last_image_stamp > ros::Duration(5.0))
        {
          ROS_ERROR("rgbd to pc did not receive images for 5 seconds ... restarting ...");
          exit(1);
        }

        rgbd::Image image;
        if (client.nextImage(image))
        {
            if (image.getDepthImage().data)
            {
                last_image_stamp = ros::Time(image.getTimestamp());

                // Convert camera info to message
                rgbd::View view(image, image.getDepthImage().cols);

                 // Create point cloud
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_msg(new pcl::PointCloud<pcl::PointXYZRGB>());

                 pc_msg->header.stamp = image.getTimestamp() * 1e6;
                 pc_msg->header.frame_id = image.getFrameId();
                 pc_msg->width  = 0;
                 pc_msg->height  = 1;
                 pc_msg->is_dense = true;

                 // Fill point cloud
                 for(int y = 0; y < view.getHeight(); ++y)
                 {
                     for(int x = 0; x < view.getWidth(); ++x)
                     {
                         geo::Vector3 p;
                         if (view.getPoint3D(x, y, p))
                         {
                             const cv::Vec3b& c = view.getColor(x, y);

                             // Push back and correct for geolib frame
                             pc_msg->points.push_back(pcl::PointXYZRGB());
                             pcl::PointXYZRGB& p_pcl = pc_msg->points.back();
                             p_pcl.x = p.x; p_pcl.y = -p.y; p_pcl.z = -p.z;
                             p_pcl.r = c[2]; p_pcl.g = c[1]; p_pcl.b = c[0];
                             pc_msg->width++;
 //                            std::cout << p << std::endl;
                         }
                         else
                         {
 //                            pc_msg->points.push_back(pcl::PointXYZ(0, 0, 0));
 //                            pc_msg->width++;
                             pc_msg->is_dense = false;
                         }
                     }
                 }

                 // Publish
                 pub_depth_pc.publish(pc_msg);
            }
        }

        r.sleep();
    }

    return 0;
}
