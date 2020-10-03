# Test

This file contains information about the test nodes and a correct procedure to test the
communication.

## Test Nodes

The nodes are split into two types, server and client nodes.

The server nodes generate an artificial image and distribute it via their specific way of
communication. The client nodes use their specific way of communication to receive an image
and show it on a gui. The server and client nodes with matching names should always communicate with
each other.

Server nodes:

* `rgbd_test_server_ros`: Publishes an image via default ROS image message with the use of the
`ServerROS` class.
* `rgbd_test_server_rgbd`: Publishes an image via the custom RGBD message on a ROS topic with the
use of the `ServerRGBD` class. This also provides the RGBD service.
* `rgbd_test_server_shm`: Publishes an image via shared memory communication with the use of
the `ServerSHM` class.
* `rgbd_test_server`: Provides the interfaces of the `Server` class. This includes the interface of
`ServerRGBD` and `ServrSHM` combined with a hostname topic.

Client nodes:

* `rgbd_test_client_ros`: Subsribes to the default ROS image topics to receive an image with the
use of the `ClientROS` class.
* `rgbd_test_client_rgbd`: Subsribes to the custom RGBD topic with the
use of the `ClientRGBD` class.
* `rgbd_test_client_shm`: Receives an image via shared memory communication with the use of
the `ClientSHM` class. __Note__: This node has an initialization timeout. The shared memory needs
needs to be created before this timeout ends. So start the server node providing the shared memory
either before or directly after starting this node.
* `rgbd_test_client`: Receives images via either the RGBD topic or shared memory communication with
the use of the `Client` class. It switched between these two based on a hostname topic to determine
the availability of shared memory communication, but uses RGBD as a fallback.

## Test procedure

As mentioned earlier, the server and client nodes with matching names should always communicate
with each other.

The nodes communicating via ROS topics, `*_ros` and `*_rgbd`, can be started, closed and restarted
in any order. The nodes should always reconnect automatically and the client node should show new
images. Test this by opening the server before the client and vice-versa. Test the reconnection by
closing the first node and restart it again. Try mutltiple time intervals.

The bare shared memory nodes, `rgbd_test_server_shm` and `rgbd_test_client_shm`, are not that
robust by themselves. These don't reconnect and the shared memory needs to be opened before the
timoeut of the client nodes has ended. Otherwise no communication will be established.
Test this by opening the server before the client or just directly after.

The robust `rgbd_test_server` and `rgbd_test_client` should always reconnect. Apply the same test
procedure as the nodes using ROS topics. When both are running on the same machine, communication
should be happening via the shared memory. This can be checked by either `rostopic info` or
`rosnode info`.

### Cross testing

To verify the robust behaviour of the classes perform the following tests:

1. Combine the `rgbd_test_client` with  `rgbd_test_server_rgbd`. This should always reconnect.
2. Extend the first test by starting a `rgbd_to_shm` node. (__Note__: This node should be run on
the same machine as the client node. This might require some remapping of the RGBD topic.) The
client node should switch from RGBD communication to shared memory communication. By closing the
`rgbd_to_shm` node, the client node should switch back to the RGBD topic. After both opening and
closing of the `rgbd_to_shm` node, check the way of communication again via either `rostopic info`
or `rosnode info`.
3. Start the `rgbd_test_server` followed by the `rgbd_test_client_shm`. Apply the same test as
used by the `rgbd_test_server_shm`. As `rgbd_test_server` provides the same shared memory
communication and extra RGBD topic and service.
4. In any test, the `rgbd_test_server` can be replaced by the combination of the
`rgbd_test_server_ros` with the `ros_to_rgbd` nodes.
5. In any test, the `rgbd_test_client` can be replaced by the combination of the `rgbd_to_ros` and
the `rgbd_test_client_ros` nodes.
