#include "test_client_templ.h"
#include "rgbd/client.h"

int main(int argc, char **argv)
{
    return main_templ<rgbd::Client, true>(argc, argv);
}
