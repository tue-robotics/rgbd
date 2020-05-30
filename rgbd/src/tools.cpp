#include "rgbd/tools.h"

#include <unistd.h>
#include <limits.h>


namespace rgbd
{

std::string get_hostname()
{
    char hostname[HOST_NAME_MAX];
    if (gethostname(hostname, HOST_NAME_MAX) != 0)
        return std::string();

    return std::string(hostname);
}

}
