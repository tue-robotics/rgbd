#include "rgbd/client_shm.h"
#include "test_client_templ.h"

int main(int argc, char** argv)
{
    return main_templ<rgbd::ClientSHM>(argc, argv);
}
