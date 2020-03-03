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

    uint32_t rgb_width;         // width of rgb image
    uint32_t rgb_height;        // height of rgb image
    uint32_t depth_width;       // width of depth image
    uint32_t depth_height;      // height of depth image

    double timestamp;
    double fx, fy, cx, cy, tx, ty; // intrinsic cam parameters
    char frame_id[1000]; // TODO
};

} // end namespace rgbd

#endif
