#ifndef RGBD_TOOLS_H_
#define RGBD_TOOLS_H_

#include <string>

namespace rgbd {

/**
 * @brief Get the hostname of the current machine
 * @return hostname or an empty string in case of any error
 */
std::string get_hostname();

}

#endif // RGBD_TOOLS_H_
