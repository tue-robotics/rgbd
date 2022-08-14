#include "test_client_templ.h"
#include "rgbd/client_shm.h"

int main(int argc, char **argv)
{
    return main_templ<rgbd::ClientSHM, true>(argc, argv);
}
