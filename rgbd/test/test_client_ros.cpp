#include "test_client_templ.h"
#include "rgbd/client_ros.h"

class TestClientROS : public rgbd::ClientROS
{
public:
    TestClientROS() {}

    virtual ~TestClientROS() {}

    bool initialize(std::string server_name);
};

bool TestClientROS::initialize(std::string server_name)
{
    std::size_t last_slash = server_name.rfind("/");
    if (last_slash != std::string::npos)
    {
        std::string server_ns = server_name.substr(0, last_slash);
        return ClientROS::initialize(server_ns + "/rgb/image", server_ns + "/depth/image", server_ns + "/rgb/camera_info");
    }
    else
    {
        return ClientROS::initialize("rgb/image", "depth/image", "rgb/camera_info");
    }
}

int main(int argc, char **argv)
{
    return main_templ<TestClientROS>(argc, argv);
}
