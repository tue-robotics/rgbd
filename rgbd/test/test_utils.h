#ifndef RGBD_TEST_UTILS_H_
#define RGBD_TEST_UTILS_H_

#include <opencv2/core.hpp>

#include <rgbd/image.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

namespace rgbd {

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

    return rgbd::Image(rgb_image, depth_image, cam_model, "test_frame_id", 5);
}

}

#endif // RGBD_TEST_UTILS_H_
