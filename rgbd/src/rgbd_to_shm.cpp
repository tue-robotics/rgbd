#include <rclcpp/rclcpp.hpp>

#include "rgbd/client_rgbd.h"
#include "rgbd/image.h"
#include "rgbd/server_shm.h"
#include "rgbd/utility.h"

#include <memory>
#include <thread>

class Node
{
public:
    explicit Node(const rclcpp::Node::SharedPtr& node)
        : node_(node)
        , client_(node_)
        , server_(node_)
        , rate_(30)
        , server_name_("rgbd")
        , host_name_(rgbd::get_hostname())
    {
        rate_ = node_->declare_parameter<double>("rate", 30.0);

        client_.initialize(server_name_);
        server_.initialize(server_name_);
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
                    pub_hostname_thread_ptr_ = std::make_unique<std::thread>(rgbd::pubHostnameThreadFunc, node_, server_name_, host_name_, 10.0f);
                server_.send(*image_ptr);
            }
            rclcpp::spin_some(node_);
            r.sleep();
        }

        return 0;
    }

private:
    rclcpp::Node::SharedPtr node_;

    rgbd::ClientRGBD client_;
    rgbd::ServerSHM server_;

    double rate_;

    const std::string server_name_;
    const std::string host_name_;

    std::unique_ptr<std::thread> pub_hostname_thread_ptr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("rgbd_to_shm");
    Node app(node);

    const int rc = app.run();
    rclcpp::shutdown();
    return rc;
}
