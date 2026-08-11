#ifndef RGBD_TYPES_H_
#define RGBD_TYPES_H_

#include <memory>

namespace rgbd
{

class Image;
using ImagePtr = std::shared_ptr<Image>;
using ImageConstPtr = std::shared_ptr<const Image>;

} // namespace rgbd

#endif // RGBD_TYPES_H_
