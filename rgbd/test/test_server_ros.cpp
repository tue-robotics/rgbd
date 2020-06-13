#include "test_server_templ.h"
#include "rgbd/server_ros.h"

class TestServerROS : public rgbd::ServerROS
{
public:
    TestServerROS() {}

    virtual ~TestServerROS() {}

    void initialize(std::string server_name);
};

void TestServerROS::initialize(std::string server_name)
{
    std::size_t last_slash = server_name.rfind("/");
    if (last_slash != std::string::npos)
    {
        std::string server_ns = server_name.substr(0, last_slash);
        ServerROS::initialize(server_ns, true, true, true);
    }
    else
    {
        ServerROS::initialize();
    }
}

int main(int argc, char **argv)
{
    return main_templ<TestServerROS>(argc, argv);
}
