#include "rgbd/utility.h"

#include <bits/posix1_lim.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <climits>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace rgbd
{

std::string getHostname()
{
    char hostname[HOST_NAME_MAX];
    if (gethostname(hostname, HOST_NAME_MAX) != 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("utility"), "Can't determine hostname");
        throw std::runtime_error("Can't determine hostname");
    }

    return {hostname};
}

} // namespace rgbd
