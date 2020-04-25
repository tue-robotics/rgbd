#ifndef RGBD_IMAGE_HEADER_H_
#define RGBD_IMAGE_HEADER_H_

#include <stdint.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

namespace rgbd
{

struct BufferHeader
{
    uint64_t sequence_nr;       // sequence number of the image (can be used to check if there is a new image)

    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex      mutex;

    //Condition to wait when the queue is empty
    boost::interprocess::interprocess_condition  cond_empty;

    //Condition to wait when the queue is full
    boost::interprocess::interprocess_condition  cond_full;

    int rgb_width;         // width of rgb image
    int rgb_height;        // height of rgb image
    int depth_width;       // width of depth image
    int depth_height;      // height of depth image

    // CameraInfo
    // CameraInfo/header
    double timestamp;
    char frame_id[1000];
    // CameraInfo main
    uint32_t height, width, binning_x, binning_y;
    char distortion_model[1000];
    size_t size_D; // Max 5
    double D[5], K[9], R[9], P[12]; // D limited to 5, https://www.ros.org/reps/rep-0104.html#alternate-distortion-models
    // CameraInfo/roi
    uint32_t roi_x_offset, roi_y_offset, roi_height, roi_width;
    bool roi_do_rectify;
};

} // end namespace rgbd

#endif // RGBD_IMAGE_HEADER_H_
