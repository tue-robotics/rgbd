#include "rgbd/server_rgbd.h"
#include "test_server_templ.h"

int main(int argc, char** argv)
{
    return main_templ<rgbd::ServerRGBD>(argc, argv);
}
