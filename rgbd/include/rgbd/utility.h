#ifndef RGBD_UTILITY_H_
#define RGBD_UTILITY_H_

#include <string>

namespace rgbd {

/**
 * @brief Get the hostname of the current machine
 * @return hostname or an empty string in case of any error
 */
std::string get_hostname();

}

#endif // RGBD_UTILITY_H_
