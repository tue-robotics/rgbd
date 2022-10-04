#include <ros/init.h>

#include <rgbd/server_shm.h>
#include <rgbd/image.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_shm");
    ros::start();

    rgbd::ServerSHM server1;
    rgbd::ServerSHM server2;

    server1.initialize("~bla");
    server2.initialize("~bla");

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

    server1.send(image);
    server2.send(image);

    return 0;
}
