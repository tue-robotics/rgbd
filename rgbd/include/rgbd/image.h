#ifndef RGBD_IMAGE_H_
#define RGBD_IMAGE_H_

#include "rgbd/types.h"

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <rgbd_msgs/RGBD.h>
#include <sensor_msgs/CameraInfo.h>


namespace tue
{
namespace serialization
{
    class InputArchive;
    class OutputArchive;
}
}

namespace rgbd {

enum CameraModelType
{
    CAMERA_MODEL_NONE = 0,
    CAMERA_MODEL_PINHOLE = 1
};

enum RGBStorageType
{
    RGB_STORAGE_NONE = 0,
    RGB_STORAGE_LOSSLESS = 1,
    RGB_STORAGE_JPG = 2
};

enum DepthStorageType
{
    DEPTH_STORAGE_NONE = 0,
    DEPTH_STORAGE_LOSSLESS = 1,
    DEPTH_STORAGE_PNG = 2
};

class Image {

    friend class ClientSHM;

public:

    /**
     * @brief Empty constructor
     */
    Image();

    /**
     * @brief Full constructor
     * @param rgb_image Resulting RGB color image.
     * @param depth_image Resulting depth image.
     * @param cam_model Camera model used to take the image.
     * @param frame_id Frame_id in which the image is taken.
     * @param timestamp timestamp of the image.
     */
    Image(const cv::Mat& rgb_image,
          const cv::Mat& depth_image,
          const image_geometry::PinholeCameraModel& cam_model,
          const std::string& frame_id,
          double timestamp);

    /**
     * @brief Get the depth image.
     * @return Const reference to the depth image.
     */
    inline const cv::Mat& getDepthImage() const { return depth_image_; }

    /**
     * @brief Get the depth image.
     * @return reference to the depth image.
     */
    inline cv::Mat& getDepthImage() { return depth_image_; }

    /**
     * @brief Get the RGB color image.
     * @return Const reference to the RGB color image.
     */
    inline const cv::Mat& getRGBImage() const { return rgb_image_; }

    /**
     * @brief Get the BGR color image.
     * @return reference to the BGR color image.
     */
    inline cv::Mat& getRGBImage() { return rgb_image_; }

    /**
     * @brief Get the frame_id.
     * @return Const reference to the frame_id.
     */
    inline const std::string& getFrameId() const { return frame_id_; }

    /**
     * @brief Get the timestamp.
     * @return Copy of the timestamp.
     */
    inline double getTimestamp() const { return timestamp_; }

    /**
     * @brief Get the camera model.
     * @return Const reference to the camera model.
     */
    inline const image_geometry::PinholeCameraModel& getCameraModel() const
    {
        return cam_model_;
    }

    /**
     * @brief Set the depth image.
     * @param depth_image The depth image to set.
     */
    inline void setDepthImage(const cv::Mat& depth_image) { depth_image_ = depth_image; }

    /**
     * @brief Set the BGR color image.
     * @param depth_image The BGR color image to set.
     */
    inline void setRGBImage(const cv::Mat& rgb_image) { rgb_image_ = rgb_image; }

    /**
     * @brief Set the frame_id.
     * @param frame_id The frame_id to set.
     */
    inline void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }

    /**
     * @brief Set the timestamp.
     * @param timestamp The timestamp to set.
     */
    inline void setTimestamp(double timestamp) { timestamp_ = timestamp; }

    /**
     * @brief Set the camera model by using a camera info message.
     * The frame_id and timestamp of the message will be emptied.
     * @param cam_info The camera info message to construct the camera model.
     */
    void setCameraInfo(sensor_msgs::CameraInfo cam_info);

    /**
     * @brief Set the camera model.
     * The frame_id and timestamp in the camera info inside the camera model will be emptied.
     * @param cam_model The camera model to set.
     */
    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model);

    /**
     * @brief Create a clone of this image.
     * The depth and RGB images will be cloned, so the new image will not share data.
     * @return New image.
     */
    Image clone() const;

    /**
     * @brief Equality operator
     * @param other Other image to compare with.
     * @return equality
     */
    bool operator==(const rgbd::Image& other) const;

    /**
     * @brief Non-equality operator
     * @param other Other image to compare with.
     * @return non-equality
     */
    inline bool operator!=(const rgbd::Image& other) const { return !(*this == other); }

    friend std::ostream& operator<< (std::ostream& out, const rgbd::Image& image);

    friend bool serialize(const Image& image, tue::serialization::OutputArchive& a,
                          RGBStorageType rgb_type,
                          DepthStorageType depth_type);

    friend bool deserialize(tue::serialization::InputArchive& a, Image& image);

    friend bool convert(const rgbd_msgs::RGBDConstPtr& msg, rgbd::Image*& image);

protected:

    cv::Mat rgb_image_; // BGR format
    cv::Mat depth_image_;

    /**
     * @brief Camera model
     * Header in camera info should always be empty to prevent inconsistent behaviour of equality operators.
     */
    image_geometry::PinholeCameraModel cam_model_;

    std::string frame_id_;
    double timestamp_;

};

}

#endif // RGBD_IMAGE_H_
