#ifndef RGBD_TEST_UTILS_H_
#define RGBD_TEST_UTILS_H_

#include <opencv2/core.hpp>

#include <rgbd/image.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <random>
#include <string>

namespace rgbd {

std::string randomString(size_t length=10)
{
    static std::mt19937 generator{std::random_device{}()};
   //modify range according to your need "A-Z","a-z" or "0-9" or whatever you need.
    static std::uniform_int_distribution<int> distribution{'a', 'z'};
    std::string rand_str(length, '\0');
    for (char& dis: rand_str)
        dis = distribution(generator);

    return rand_str;
}

double randomDouble(double min=0, double max=1000000)
{
    static std::default_random_engine e;
    std::uniform_real_distribution<double> dis(min, max); // range min - max
    return dis(e);
}

rgbd::Image generateRandomImage()
{
    cv::Mat rgb_image(480, 640, CV_8UC3);
    cv::randu(rgb_image, 0, 255);
    cv::Mat depth_image(480, 640, CV_32FC1);
    cv::randu(depth_image, 0., 100.);
    sensor_msgs::CameraInfo cam_info;
    cam_info.D.resize(5, 0.0);
    cam_info.K = {554.2559327880068, 0.0, 320.5,
                  0.0, 554.2559327880068, 240.5,
                  0.0, 0.0, 1.0};
    cam_info.R.fill(0.0);
    cam_info.R[0] = 1.0;
    cam_info.R[4] = 1.0;
    cam_info.R[8] = 1.0;
    cam_info.P = {554.2559327880068, 0.0, 320.5, 0.0,
                  0.0, 554.2559327880068, 240.5, 0.0,
                  0.0, 0.0, 1.0, 0.0};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = 640;
    cam_info.height = 480;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);

    return rgbd::Image(rgb_image, depth_image, cam_model, randomString(), randomDouble());
}

}

#endif // RGBD_TEST_UTILS_H_
