#include "test_server_templ.h"
#include "rgbd/server.h"

int main(int argc, char **argv)
{
    return main_templ<rgbd::Server>(argc, argv);
}
