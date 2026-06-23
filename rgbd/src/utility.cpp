#include "rgbd/utility.h"

#include <rclcpp/rclcpp.hpp>

#include <limits.h>
#include <unistd.h>

namespace rgbd
{

std::string get_hostname()
{
    char hostname[HOST_NAME_MAX];
    if (gethostname(hostname, HOST_NAME_MAX) != 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("utility"), "Can't determine hostname");
        throw std::runtime_error("Can't determine hostname");
    }

    return std::string(hostname);
}

} // namespace rgbd
