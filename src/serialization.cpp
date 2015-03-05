#include "rgbd/serialization.h"
#include "rgbd/Image.h"

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <opencv2/highgui/highgui.hpp>

namespace rgbd
{

const static int SERIALIZATION_VERSION = 1;

// ----------------------------------------------------------------------------------------------------
//
//                                          SERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

bool serialize(const Image& image, tue::serialization::OutputArchive& a,
               RGBStorageType rgb_type, DepthStorageType depth_type)
{
    // - - - - - - - - - - - - - - - - GENERAL INFO - - - - - - - - - - - - - - - -

    a << SERIALIZATION_VERSION;

    a << image.getFrameId();
    a << image.getTimestamp();

    // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

    const image_geometry::PinholeCameraModel& cam_model = image.cam_model_;

    if (cam_model.initialized())
    {
        a << CAMERA_MODEL_PINHOLE;
        a << cam_model.fx() << cam_model.fy();
        a << cam_model.cx() << cam_model.cy();
        a << cam_model.Tx() << cam_model.Ty();
    }
    else
    {
//        a << CAMERA_MODEL_NONE;
        const geo::DepthCamera& rasterizer = image.rasterizer_;

        a << CAMERA_MODEL_PINHOLE;
        a << rasterizer.getFocalLengthX() << rasterizer.getFocalLengthY();
        a << rasterizer.getOpticalCenterX() << rasterizer.getOpticalCenterY();
        a << rasterizer.getOpticalTranslationX() << rasterizer.getOpticalTranslationY();
    }

    // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

    if (!image.rgb_image_.data)
        rgb_type = RGB_STORAGE_NONE;

    a << rgb_type;

    if (rgb_type == RGB_STORAGE_NONE)
    {
    }
    else if (rgb_type == RGB_STORAGE_LOSSLESS)
    {
        a << image.rgb_image_.cols;
        a << image.rgb_image_.rows;

        int size = image.rgb_image_.rows * image.rgb_image_.cols * 3;
//        for(int i = 0; i < size; ++i)
//            a << image.rgb_image_.data[i];
        a.write((const char*)image.rgb_image_.data, size);
    }
    else if (rgb_type == RGB_STORAGE_JPG)
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        std::vector<unsigned char> rgb_data;

        // Compress image
        if (!cv::imencode(".jpg", image.rgb_image_, rgb_data, rgb_params)) {
            std::cout << "RGB image compression failed" << std::endl;
            return false;
        }

        a << (int)rgb_data.size();
//        for(unsigned int i = 0; i < rgb_data.size(); ++i)
//        {
//            a << rgb_data[i];
//        }
        a.write((const char*)&rgb_data[0], rgb_data.size());
    }
    else
    {
        std::cout << "Unsupported RGB STORAGE TYPE" << std::endl;
        return false;
    }

    // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

    if (!image.depth_image_.data)
        depth_type = DEPTH_STORAGE_NONE;

    a << depth_type;

    if (depth_type == DEPTH_STORAGE_NONE)
    {
    }
    else if (depth_type == DEPTH_STORAGE_LOSSLESS)
    {
        a << image.depth_image_.cols;
        a << image.depth_image_.rows;

        int size = image.depth_image_.rows * image.depth_image_.cols * 4;
//        for(int i = 0; i < size; ++i)
//            a << image.depth_image_.data[i];
        a.write((const char*)image.depth_image_.data, size);
    }
    else if (depth_type == DEPTH_STORAGE_PNG)
    {
        float depthZ0 = 100; //config_.depth_quantization;
        float depthMax = 10; //config_.depth_max;

        float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
        float depthQuantB = 1.0f - depthQuantA / depthMax;

        a << depthQuantA << depthQuantB;

        const cv::Mat& depth_image = image.depth_image_;
        cv::Mat invDepthImg(depth_image.size(), CV_16UC1);

        // Matrix iterators
        cv::MatConstIterator_<float> itDepthImg = depth_image.begin<float>(),
                                 itDepthImg_end = depth_image.end<float>();
        cv::MatIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>(),
                                     itInvDepthImg_end = invDepthImg.end<unsigned short>();

        // Quantization
        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
            // check for NaN & max depth
            if (*itDepthImg < depthMax){
                *itInvDepthImg = depthQuantA / *itDepthImg + depthQuantB;
            } else{
                *itInvDepthImg = 0;
            }
        }

        // Compression settings
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = CV_IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        std::vector<unsigned char> depth_data;

        if (!cv::imencode(".png", invDepthImg, depth_data, params)) {
            std::cout << "Depth image compression failed" << std::endl;
            return false;
        }

        a << (int)depth_data.size();
//        for(unsigned int i = 0; i < depth_data.size(); ++i)
//        {
//            a << depth_data[i];
//        }
        a.write((const char*)&depth_data[0], depth_data.size());
    }
    else
    {
        std::cout << "Unsupported DEPTH STORAGE TYPE" << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------
//
//                                         DESERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::serialization::InputArchive& a, Image& image)
{
    // - - - - - - - - - - - - - - - - GENERAL INFO - - - - - - - - - - - - - - - -

    int version;
    a >> version;

    a >> image.frame_id_;
    a >> image.timestamp_;

    // - - - - - - - - - - - - - - - - CAMERA INFO - - - - - - - - - - - - - - - -

    int cam_type;
    a >> cam_type;

    if (cam_type == CAMERA_MODEL_NONE)
    {
    }
    else if (cam_type == CAMERA_MODEL_PINHOLE)
    {
        double fx, fy, cx, cy, tx, ty;
        a >> fx >> fy;
        a >> cx >> cy;
        a >> tx >> ty;

        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.D.resize(5, 0.0);
        cam_info_msg.K.fill(0.0);
        cam_info_msg.K[0] = fx;  // fx
        cam_info_msg.K[2] = cx;  // cx
        cam_info_msg.K[4] = fy;  // fy
        cam_info_msg.K[5] = cy;  // cy
        cam_info_msg.K[8] = 1.0;

        cam_info_msg.R.fill(0.0);
        cam_info_msg.R[0] = 1.0;
        cam_info_msg.R[4] = 1.0;
        cam_info_msg.R[8] = 1.0;

        cam_info_msg.P.fill(0.0);
        cam_info_msg.P[0] = fx;  // fx
        cam_info_msg.P[2] = cx;  // cx
        cam_info_msg.P[3] = tx;  // Tx
        cam_info_msg.P[5] = fy;  // fy
        cam_info_msg.P[6] = cy;  // cy
        cam_info_msg.P[7] = ty;  // Ty
        cam_info_msg.P[10] = 1.0;

        cam_info_msg.distortion_model = "plumb_bob";
//        cam_info_msg.width = image_ptr_->rgb_image_.cols;
//        cam_info_msg.height = image_ptr_->rgb_image_.rows;

        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info_msg);
        image.cam_model_ = cam_model;

        image.setupRasterizer();
    }
    else
    {
        std::cout << "rgbd::deserialize: Unsupported camera model" << std::endl;
        return false;
    }

    // - - - - - - - - - - - - - - - - RGB IMAGE - - - - - - - - - - - - - - - -

    int rgb_type;
    a >> rgb_type;

    if (rgb_type == RGB_STORAGE_NONE)
    {
    }
    else if (rgb_type == RGB_STORAGE_LOSSLESS)
    {
        int width, height;
        a >> width;
        a >> height;

        int size = width * height * 3;
        image.rgb_image_ = cv::Mat(height, width, CV_8UC3);
        for(int i = 0; i < size; ++i)
            a >> image.rgb_image_.data[i];
    }
    else if (rgb_type == RGB_STORAGE_JPG)
    {
        int rgb_size;
        a >> rgb_size;

        std::vector<unsigned char> rgb_data(rgb_size);
        for(int i = 0; i < rgb_size; ++i)
            a >> rgb_data[i];

        image.rgb_image_ = cv::imdecode(cv::Mat(rgb_data), CV_LOAD_IMAGE_UNCHANGED);
    }
    else
    {
        std::cout << "rgbd::deserialize: Unsupported rgb storage format: " << rgb_type << std::endl;
        return false;
    }

    // - - - - - - - - - - - - - - - - DEPTH IMAGE - - - - - - - - - - - - - - - -

    int depth_type;
    a >> depth_type;

    if (depth_type == DEPTH_STORAGE_NONE)
    {
    }
    else if (depth_type == DEPTH_STORAGE_LOSSLESS)
    {
        int width, height;
        a >> width;
        a >> height;

        int size = width * height * 4;
        image.depth_image_ = cv::Mat(height, width, CV_32FC1);
        for(int i = 0; i < size; ++i)
            a >> image.depth_image_.data[i];
    }
    else if (depth_type == DEPTH_STORAGE_PNG)
    {
        float depthQuantA, depthQuantB;
        a >> depthQuantA >> depthQuantB;

        int depth_size;
        a >> depth_size;

        std::vector<unsigned char> depth_data(depth_size);
        for(int i = 0; i < depth_size; ++i)
            a >> depth_data[i];

        cv::Mat decompressed = cv::imdecode(depth_data, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat& depth_image = image.depth_image_;
        depth_image = cv::Mat(decompressed.size(), CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> itDepthImg = depth_image.begin<float>(),
                itDepthImg_end = depth_image.end<float>();
        cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
            // check for NaN & max depth
            if (*itInvDepthImg) {
                *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
            } else {
                *itDepthImg = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    else
    {
        std::cout << "rgbd::deserialize: Unsupported depth storage format: " << depth_type << std::endl;
        return false;
    }

    return true;
}

}
