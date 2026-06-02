#include "rgbd/client.h"

#include "rgbd/utility.h"

namespace rgbd {

Client::Client(const rclcpp::Node::SharedPtr& node)
    : node_(node ? node : rclcpp::Node::make_shared("rgbd_client"))
    , sub_shm_hosts_(nullptr)
    , cb_group_shm_hosts_(nullptr)
    , client_rgbd_(node_)
    , last_time_shm_server_online_(0, 0, RCL_ROS_TIME)
    , stop_sub_hosts_thread_(false)
    , client_impl_mode_(ClientImplMode::rgbd)
{
    hostname_ = get_hostname();
}

Client::~Client()
{
    deinitialize();
}

bool Client::initialize(const std::string& server_name, float)
{
    if (initialized() && server_name_ == server_name)
    {
        return true;
    }
    else if (initialized())
    {
        deinitialize();
    }

    cb_group_shm_hosts_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_shm_hosts_;

    sub_shm_hosts_ = node_->create_subscription<std_msgs::msg::String>(
        server_name + "/hosts",
        10,
        std::bind(&Client::hostsCallback, this, std::placeholders::_1), sub_options);

    stop_sub_hosts_thread_ = false;
    sub_hosts_thread_ = std::thread(&Client::subHostsThreadFunc, this, 20.0f);

    server_name_ = server_name;
    return true;
}

bool Client::deinitialize()
{
    if (!initialized())
    {
        return true;
    }

    sub_shm_hosts_.reset();
    stop_sub_hosts_thread_ = true;

    if (sub_hosts_thread_.joinable())
    {
        sub_hosts_thread_.join();
    }

    if (client_shm_.initialized())
    {
        client_shm_.deinitialize();
    }
    if (client_rgbd_.initialized())
    {
        client_rgbd_.deinitialize();
    }

    last_time_shm_server_online_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    server_name_.clear();

    return true;
}

bool Client::nextImage(Image& image)
{
    std::lock_guard<std::mutex> lg(switch_impl_mutex_);
    if (client_impl_mode_ == ClientImplMode::shm)
    {
        return client_shm_.nextImage(image);
    }
    return client_rgbd_.nextImage(image);
}

ImagePtr Client::nextImage()
{
    std::lock_guard<std::mutex> lg(switch_impl_mutex_);
    if (client_impl_mode_ == ClientImplMode::shm)
    {
        return client_shm_.nextImage();
    }
    return client_rgbd_.nextImage();
}

void Client::hostsCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    if (msg->data != hostname_)
    {
        return;
    }

    last_time_shm_server_online_ = node_->now();
    RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("Client"), *node_->get_clock(), 5000, "SHM server online on: %s", hostname_.c_str());
}

void Client::subHostsThreadFunc(float frequency)
{
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Rate r(frequency);
    const double timeout = 3.0 / static_cast<double>(frequency);

    executor.add_callback_group(cb_group_shm_hosts_, node_->get_node_base_interface());

    while (rclcpp::ok() && !stop_sub_hosts_thread_)
    {
        executor.spin_some();

        if (node_->now() > (last_time_shm_server_online_ + rclcpp::Duration::from_seconds(timeout)))
        {
            std::lock_guard<std::mutex> lg(switch_impl_mutex_);
            if (client_shm_.initialized())
            {
                client_shm_.deinitialize();
            }
            if (!client_rgbd_.initialized())
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Client"), "Switching to ClientRGBD");
                client_rgbd_.initialize(server_name_);
            }
            client_impl_mode_ = ClientImplMode::rgbd;
        }
        else
        {
            std::lock_guard<std::mutex> lg(switch_impl_mutex_);
            if (client_rgbd_.initialized())
            {
                client_rgbd_.deinitialize();
            }
            if (!client_shm_.initialized())
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Client"), "Switching to ClientSHM");
                client_shm_.initialize(server_name_, 0.001);
            }
            client_impl_mode_ = ClientImplMode::shm;
        }
        r.sleep();
    }
    executor.remove_callback_group(cb_group_shm_hosts_);
}

}  // namespace rgbd
