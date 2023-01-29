#include <nodelet/nodelet.h>
#include <nodelet/detail/callback_queue.h>

#include <pluginlib/class_list_macros.hpp>

#include <ros/node_handle.h>
#include <ros/console.h>

#include "rgbd/client_ros_base.h"
#include "rgbd/image.h"
#include "rgbd/server.h"

#include <memory>


namespace rgbd
{

class ClientRosNodelet : public ClientROSBase
{
public:
    ClientRosNodelet(ros::NodeHandle& nh) : ClientROSBase(nh)
    {
    }

    ~ClientRosNodelet()
    {
    }

    const Image* getImage()
    {
        return image_ptr_;
    }

    message_filters::Synchronizer<RGBDApproxPolicy>* getSync()
    {
        return sync_.get();
    }

    using ClientROSBase::imageCallback;

};

class ROSToRGBDNodelet : public nodelet::Nodelet
{
public:
    ROSToRGBDNodelet()
    {
    }

private:
    virtual void onInit()
    {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle nh_private = getPrivateNodeHandle();

        // ----- READ RGB STORAGE TYPE

        rgbd::RGBStorageType rgb_type;

        std::string rgb_type_str = "lossless";
        nh_private.getParam("rgb_storage", rgb_type_str);
        if (rgb_type_str == "none")
            rgb_type = rgbd::RGB_STORAGE_NONE;
        else if (rgb_type_str == "lossless")
            rgb_type = rgbd::RGB_STORAGE_LOSSLESS;
        else if (rgb_type_str == "jpg")
            rgb_type = rgbd::RGB_STORAGE_JPG;
        else
        {
            ROS_ERROR("Unknown 'rgb_storage' type: should be 'none', 'lossless', or 'jpg'.");
            return;
        }

        // ----- READ DEPTH STORAGE TYPE

        rgbd::DepthStorageType depth_type;

        std::string depth_type_str = "lossless";
        nh_private.getParam("depth_storage", depth_type_str);
        if (depth_type_str == "none")
            depth_type = rgbd::DEPTH_STORAGE_NONE;
        else if (depth_type_str == "lossless")
            depth_type = rgbd::DEPTH_STORAGE_LOSSLESS;
        else if (depth_type_str == "png")
            depth_type = rgbd::DEPTH_STORAGE_PNG;
        else
        {
            ROS_ERROR("Unknown 'depth_storage' type: should be 'none', 'lossless', or 'png'.");
            return;
        }

        client_ = std::make_unique<rgbd::ClientRosNodelet>(nh);
        client_->initialize("rgb_image", "depth_image", "cam_info");

        server_ = std::make_unique<rgbd::Server>(nh);
        server_->initialize(ros::names::resolve("rgbd"), rgb_type, depth_type);

        client_->getSync()->registerCallback(boost::bind(&ROSToRGBDNodelet::imageCallback, this, _1, _2));
    }

    bool imageCallback(const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::ImageConstPtr& depth_image_msg)
    {
        if (!client_->imageCallback(rgb_image_msg, depth_image_msg))
        {
            ROS_ERROR("Error during processing the image callback. See log above.");
            return false;
        }
        server_->send(*client_->getImage());
        return true;
    }

    std::unique_ptr<rgbd::ClientRosNodelet> client_;
    std::unique_ptr<rgbd::Server> server_;
};

PLUGINLIB_EXPORT_CLASS(rgbd::ROSToRGBDNodelet, nodelet::Nodelet)

} // end namespace rgbd
