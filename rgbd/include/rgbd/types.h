#ifndef RGBD_TYPES_H_
#define RGBD_TYPES_H_

#include <boost/shared_ptr.hpp>

namespace rgbd {

class Image;
typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<const Image> ImageConstPtr;

}

#endif // RGBD_TYPES_H_
