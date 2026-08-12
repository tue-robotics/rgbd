#ifndef RGBD_ROS_TO_RGBD_COMPONENT_H_
#define RGBD_ROS_TO_RGBD_COMPONENT_H_

#include <rclcpp/rclcpp.hpp>

#include "rgbd/client_ros.h"
#include "rgbd/server.h"

#include <memory>
#include <string>

namespace rgbd
{

class RosToRGBDComponent : public rclcpp::Node
{
public:
    explicit RosToRGBDComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RosToRGBDComponent() override = default;

private:
    static RGBStorageType parseRGBStorageType(const std::string& rgb_type_str);
    static DepthStorageType parseDepthStorageType(const std::string& depth_type_str);

    bool initializeInterfaces();
    void runOnce();

    RGBStorageType rgb_type_;
    DepthStorageType depth_type_;

    bool interfaces_initialized_{false};

    std::unique_ptr<ClientROS> client_;
    std::unique_ptr<Server> server_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace rgbd

#endif // RGBD_ROS_TO_RGBD_COMPONENT_H_
