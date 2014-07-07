#ifndef RGBD_SERIALIZATION_H_
#define RGBD_SERIALIZATION_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

namespace rgbd
{

class RGBDImage;

// SERIALIZATION

void serialize(const RGBDImage& image, tue::serialization::OutputArchive& a);

// DESERIALIZATION

void deserialize(tue::serialization::InputArchive& a, RGBDImage& image);

}

#endif
