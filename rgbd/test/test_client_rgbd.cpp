#include "rgbd/client_rgbd.h"
#include "test_client_templ.h"

int main(int argc, char** argv)
{
    return main_templ<rgbd::ClientRGBD>(argc, argv);
}
