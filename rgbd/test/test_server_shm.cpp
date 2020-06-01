#include "test_server_templ.h"
#include "rgbd/server_shm.h"

int main(int argc, char **argv)
{
    return main_templ<rgbd::ServerSHM>(argc, argv);
}
