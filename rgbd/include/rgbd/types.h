#ifndef RGBD_TYPES_H_
#define RGBD_TYPES_H_

#include <memory>

namespace rgbd {

class Image;
typedef std::shared_ptr<Image> ImagePtr;
typedef std::shared_ptr<const Image> ImageConstPtr;

}

#endif // RGBD_TYPES_H_
