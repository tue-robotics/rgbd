#include "rgbd/utility.h"

#include <ros/console.h>

#include <exception>
#include <limits.h>
#include <unistd.h>


namespace rgbd
{

std::string get_hostname()
{
    char hostname[HOST_NAME_MAX];
    if (gethostname(hostname, HOST_NAME_MAX) != 0)
    {
        ROS_FATAL_NAMED("utility", "Can't determine hostname");
        throw std::runtime_error("Can't determine hostname");
    }

    return std::string(hostname);
}

}
