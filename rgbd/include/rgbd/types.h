#ifndef RGBD_TRANSPORT_TYPES_H_
#define RGBD_TRANSPORT_TYPES_H_

#include <boost/shared_ptr.hpp>

namespace rgbd {

class Image;
typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<const Image> ImageConstPtr;

}

#endif
