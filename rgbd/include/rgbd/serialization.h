#ifndef RGBD_SERIALIZATION_H_
#define RGBD_SERIALIZATION_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include "rgbd/Image.h"

namespace rgbd
{

class Image;

// SERIALIZATION

bool serialize(const Image& image, tue::serialization::OutputArchive& a,
               RGBStorageType rgb_type = RGB_STORAGE_JPG,
               DepthStorageType depth_type = DEPTH_STORAGE_PNG);

// DESERIALIZATION

bool deserialize(tue::serialization::InputArchive& a, Image& image);

}

#endif // RGBD_SERIALIZATION_H_
