#include "rgbd/server_shm.h"
#include "test_server_templ.h"

int main(int argc, char** argv)
{
    return main_templ<rgbd::ServerSHM>(argc, argv);
}
