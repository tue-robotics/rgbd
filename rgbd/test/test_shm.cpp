#include <ros/init.h>

#include <boost/interprocess/shared_memory_object.hpp>

#include <rgbd/server_shm.h>
#include <rgbd/image.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

namespace ipc = boost::interprocess;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_shm");
    ros::start();

    std::string server_name = "~bla";
    if (argc >= 2)
        server_name = std::string(argv[1]);

    ROS_INFO_STREAM("server_name: " << server_name);

    std::unique_ptr<rgbd::ServerSHM> server1 = std::make_unique<rgbd::ServerSHM>();
//    std::unique_ptr<rgbd::ServerSHM> server2 = std::make_unique<rgbd::ServerSHM>();

    server1->initialize(server_name);
//    server2->initialize(server_name);

    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
    cv::Mat depth_image(480, 640, CV_32FC1, 5.0);
    sensor_msgs::CameraInfo cam_info;
    cam_info.K = {554.2559327880068, 0.0, 320.5,
                  0.0, 554.2559327880068, 240.5,
                  0.0, 0.0, 1.0};
    cam_info.P = {554.2559327880068, 0.0, 320.5, 0.0,
                  0.0, 554.2559327880068, 240.5, 0.0,
                  0.0, 0.0, 1.0, 0.0};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = 640;
    cam_info.height = 480;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);

    rgbd::Image image(rgb_image, depth_image, cam_model, "test_frame_id", ros::Time::now().toSec());

//    ipc::shared_memory_object::remove(server_name.c_str());

    server1->send(image);
//    server2->send(image);
//    server1->send(image);
//    server2.reset();
//    server1->send(image);

    ros::WallTime start = ros::WallTime::now();
    ros::WallTime end = start + ros::WallDuration(10);
    ros::WallTime now;
    uint count = 0;
    while ((now = ros::WallTime::now()) < end)
    {
        ipc::shared_memory_object shm = ipc::shared_memory_object(ipc::open_only, server_name.c_str(), ipc::read_write);
        ++count;
    }

    ROS_INFO_STREAM("Opened and closed SHM '" << count << "' times in 10 seconds");

    return 0;
}
