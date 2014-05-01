#ifndef RGBD_TRANSPORT_TYPES_H_
#define RGBD_TRANSPORT_TYPES_H_

#include <boost/shared_ptr.hpp>

namespace rgbd {

class RGBDImage;
typedef boost::shared_ptr<RGBDImage> RGBDImagePtr;
typedef boost::shared_ptr<const RGBDImage> RGBDImageConstPtr;

}

#endif
