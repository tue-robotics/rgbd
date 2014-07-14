#ifndef RGBD_SERIALIZATION_H_
#define RGBD_SERIALIZATION_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

namespace rgbd
{

class Image;

// SERIALIZATION

void serialize(const Image& image, tue::serialization::OutputArchive& a);

// DESERIALIZATION

void deserialize(tue::serialization::InputArchive& a, Image& image);

}

#endif
