#include "rgbd_transport/Client.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_test_client");

    Client client;
    client.intialize("test");

    ros::spin();

    return 0;
}
