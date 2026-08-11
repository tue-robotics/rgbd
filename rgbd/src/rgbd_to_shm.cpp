#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>

#include "rgbd/client_rgbd.h"
#include "rgbd/server_shm.h"
#include "rgbd/types.h"
#include "rgbd/utility.h"

#include <memory>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <utility>

class Node
{
public:
    // NOLINTNEXTLINE(clang-analyzer-optin.cplusplus.UninitializedObject) flagged fields belong to boost::interprocess
    // member objects (shm_/mem_buffer_header_/mem_image_ inside server_), which are fully initialized by their own
    // default constructors.
    explicit Node(rclcpp::Node::SharedPtr node) :
        node_(std::move(node)), client_(node_), server_(node_), SERVER_NAME("rgbd"), HOST_NAME(rgbd::getHostname())
    {
        rate_ = node_->declare_parameter<double>("rate", 30.0);

        client_.initialize(SERVER_NAME);
        server_.initialize(SERVER_NAME);
    }

    virtual ~Node()
    {
        if (pub_hostname_thread_ptr_ && pub_hostname_thread_ptr_->joinable())
            pub_hostname_thread_ptr_->join();
    }

    int run()
    {
        rgbd::ImagePtr image_ptr;

        rclcpp::Rate r(rate_);
        while (rclcpp::ok())
        {
            image_ptr = client_.nextImage();
            if (image_ptr)
            {
                if (!pub_hostname_thread_ptr_)
                    pub_hostname_thread_ptr_ = std::make_unique<std::thread>(
                        rgbd::pubHostnameThreadFunc, node_, SERVER_NAME, HOST_NAME, 10.0f);
                server_.send(*image_ptr);
            }
            r.sleep();
        }

        return 0;
    }

private:
    rclcpp::Node::SharedPtr node_;

    rgbd::ClientRGBD client_;
    rgbd::ServerSHM server_;

    double rate_{30};

    const std::string SERVER_NAME;
    const std::string HOST_NAME;

    std::unique_ptr<std::thread> pub_hostname_thread_ptr_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("rgbd_to_shm");
    Node app(node);

    const int rc = app.run();
    rclcpp::shutdown();
    return rc;
}
