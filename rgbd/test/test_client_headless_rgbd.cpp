#include "test_client_templ.h"
#include "rgbd/client_rgbd.h"

int main(int argc, char **argv)
{
    return main_templ<rgbd::ClientRGBD, true>(argc, argv);
}
